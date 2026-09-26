#!/usr/bin/env python3
# Copyright 2026 Buğra Öztürk
# SPDX-License-Identifier: Apache-2.0

"""
targeting_node.py — Autonomous Shooting: HSV + Hough Circle + PID
Publishes targeting error, status, and turret commands.
"""

import time
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import CompressedImage, Image
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import Bool, String, UInt8
from cv_bridge import CvBridge

from teknofest_ika.utils.pid_controller import PIDController
from teknofest_ika.otonomi.topics import (
    TARGETING_ENABLE_TOPIC, TARGETING_STATUS_TOPIC,
    TARGETING_ERROR_TOPIC, TURRET_CMD_TOPIC,
    CAMERA_TARET_PROCESSED_TOPIC, TARGETING_DEBUG_TOPIC,
    NISAN_HSV_ALT, NISAN_HSV_UST, NISAN_HSV_ALT2, NISAN_HSV_UST2,
    NISAN_HOUGH_MAX_YARICAP,
    SHOOT_CMD_TOPIC, LASER_FIRE_DURATION, MOD_AKTIF_TOPIC,
    YOLO_CLASSES, CLASS_HEDEF_TAHTASI,
)
from teknofest_ika.otonomi.mod_yoneticisi import MOD_MANUAL

try:
    from teknofest_ika.utils.tensorrt_inferer import TensorRTInferer
    HAS_TRT = True
except Exception:
    HAS_TRT = False
try:
    from ultralytics import YOLO as UltralyticsYOLO
    HAS_ULTRALYTICS = True
except Exception:
    HAS_ULTRALYTICS = False


class TargetingNode(Node):
    def __init__(self):
        super().__init__("targeting_node")

        # Nişan halkası HSV aralığı — kalibre edilmiş değerler topics.py'de.
        self.declare_parameter("hsv_lower",  NISAN_HSV_ALT)
        self.declare_parameter("hsv_upper",  NISAN_HSV_UST)
        self.declare_parameter("hsv_lower2", NISAN_HSV_ALT2)
        self.declare_parameter("hsv_upper2", NISAN_HSV_UST2)

        # Hough Circle parameters
        self.declare_parameter("hough_dp", 1.2)
        self.declare_parameter("hough_min_dist", 50)
        self.declare_parameter("hough_param1", 100)
        self.declare_parameter("hough_param2", 30)
        self.declare_parameter("hough_min_radius", 10)
        self.declare_parameter("hough_max_radius", NISAN_HOUGH_MAX_YARICAP)

        # PID gains
        self.declare_parameter("pid_yaw", [0.5, 0.0, 0.1])
        self.declare_parameter("pid_pitch", [0.5, 0.0, 0.1])
        self.declare_parameter("output_limit", [-45.0, 45.0])  # degrees

        # Alignment & fire logic
        self.declare_parameter("align_threshold_px", 10.0)
        self.declare_parameter("fire_lock_duration_sec", 0.5)
        self.declare_parameter("fire_cooldown_sec", 2.0)
        # Üç kamera aynı USB2 hattını paylaştığında işlenmiş nişan akışı
        # ~7-9 Hz'e düşüyor (kare aralığı ~140ms) — eşik bunun altında
        # kalırsa her kare STALE_IMAGE sayılıp nişan hiç çalışmıyor.
        self.declare_parameter("image_timeout_sec", 0.5)
        self.declare_parameter("target_ema_alpha", 0.4)
        self.declare_parameter("publish_debug", True)

        # Hedef bulma yöntemi: "hsv" (kırmızı halka, Hough) veya "yolo"
        # (hedef_tahtasi sınıfı, kutu merkezine nişan). YOLO ana kamerayla
        # aynı modeli kullanır; .engine yoksa .pt'ye düşer.
        self.declare_parameter("detector", "hsv")
        self.declare_parameter("yolo_model_path", "models/best.engine")
        self.declare_parameter("yolo_conf", 0.45)
        self.declare_parameter("flip_180", False)   # kamera ters monteliyse
        # Hedef yokken taret süpürerek aramaz, bulunduğu yerde SABİT durur;
        # hedef görüş alanına girince kilitlenip ateşler. Süpürme istenirse aç.
        self.declare_parameter("sweep", False)
        # Kilit histerezisi: align_threshold_px içine girince kilitlenir, ancak
        # bu çarpanla genişletilmiş eşiği AŞINCA kilit bozulur — sınırda
        # seğirmeyi (chatter) engeller.
        self.declare_parameter("unlock_factor", 1.8)
        # Kontrol yön işareti: taret hedeften KAÇIYORSA (pozitif geri besleme)
        # ilgili ekseni -1 yap. Kamera montajı + flip + köprü PAN_YON birleşimi
        # işareti belirler; geometriden kestirmek yerine sahada doğrulanır.
        self.declare_parameter("yaw_sign", 1.0)
        self.declare_parameter("pitch_sign", 1.0)
        # Servo beslemesi iki servoyu BİRDEN kaldıramıyor (BEC eksik) — ikisi
        # aynı anda komut alınca pan aç kalıp hiç dönmüyordu. Sıralı mod: önce
        # pan ortalanır (tilt durur), sonra tilt (pan durur); tek servo tek
        # başına güçle dönebiliyor (sahada kanıtlı). BEC takılınca kapatılabilir.
        self.declare_parameter("tek_eksen", True)

        # Bağımsız otonom taret modu (misyon_fsm olmadan saha testi/demo).
        # Varsayılanlar kapalı — yarışma yolunda enable/atış misyon_fsm'de
        # kalır, bu parametreler yalnız taret_otonom.launch.py'de açılır.
        self.declare_parameter("auto_enable", False)  # açılışta hedef aramaya başla
        self.declare_parameter("auto_fire", False)    # ALIGNED olunca /shoot_command darbesi
        # Kumandanın mod anahtarına bağlanır: OTONOM'a alınınca hedef arama
        # başlar, MANUEL'e dönünce durur (stick kontrolü köprüde devralır).
        self.declare_parameter("mod_takip", False)

        self.hsv_lower = np.array(self.get_parameter("hsv_lower").value, dtype=np.uint8)
        self.hsv_upper = np.array(self.get_parameter("hsv_upper").value, dtype=np.uint8)
        self.hsv_lower2 = np.array(self.get_parameter("hsv_lower2").value, dtype=np.uint8)
        self.hsv_upper2 = np.array(self.get_parameter("hsv_upper2").value, dtype=np.uint8)

        self.hough_dp = self.get_parameter("hough_dp").value
        self.hough_min_dist = self.get_parameter("hough_min_dist").value
        self.hough_param1 = self.get_parameter("hough_param1").value
        self.hough_param2 = self.get_parameter("hough_param2").value
        self.hough_min_radius = self.get_parameter("hough_min_radius").value
        self.hough_max_radius = self.get_parameter("hough_max_radius").value

        pid_yaw_vals = self.get_parameter("pid_yaw").value
        pid_pitch_vals = self.get_parameter("pid_pitch").value
        out_limit = tuple(self.get_parameter("output_limit").value)

        self.pid_yaw = PIDController(Kp=pid_yaw_vals[0], Ki=pid_yaw_vals[1],
                                     Kd=pid_yaw_vals[2], output_limit=out_limit)
        self.pid_pitch = PIDController(Kp=pid_pitch_vals[0], Ki=pid_pitch_vals[1],
                                       Kd=pid_pitch_vals[2], output_limit=out_limit)

        self.align_threshold = self.get_parameter("align_threshold_px").value
        self.fire_lock_dur = self.get_parameter("fire_lock_duration_sec").value
        self.fire_cooldown = self.get_parameter("fire_cooldown_sec").value
        self.image_timeout = self.get_parameter("image_timeout_sec").value
        self.target_ema_alpha = self.get_parameter("target_ema_alpha").value
        self.publish_debug = self.get_parameter("publish_debug").value

        self.bridge = CvBridge()
        self.flip_180 = bool(self.get_parameter("flip_180").value)
        self.sweep = bool(self.get_parameter("sweep").value)
        self.unlock_factor = float(self.get_parameter("unlock_factor").value)
        self.yaw_sign = float(self.get_parameter("yaw_sign").value)
        self.pitch_sign = float(self.get_parameter("pitch_sign").value)
        self.tek_eksen = bool(self.get_parameter("tek_eksen").value)
        self._locked = False   # histerezis durumu
        self.detector = str(self.get_parameter("detector").value).lower()
        self._trt = None
        self._ul = None
        if self.detector == "yolo":
            self._yolo_yukle()
        self._status = "STANDBY"
        self._aligned_since = None
        self._search_yaw = 0.0
        self._search_dir = 1.0
        self._search_speed = 5.0  # deg/s
        self._latest_image = None
        self._latest_image_stamp = None
        self._last_processed_stamp = None
        self._target_ema = None  # [cx, cy] smoothed target position
        self._prev_target_found = False
        self.auto_enable = bool(self.get_parameter("auto_enable").value)
        self.auto_fire = bool(self.get_parameter("auto_fire").value)
        self.mod_takip = bool(self.get_parameter("mod_takip").value)

        self._enabled = self.auto_enable  # /targeting/enable ile kontrol edilir
        self._was_enabled = False         # geçiş algılama için

        # auto_fire darbe durumu: yanma başlangıcı ve son atış zamanı [s].
        self._atis_baslangic = None
        self._son_atis = None

        self.pub_error = self.create_publisher(Point, TARGETING_ERROR_TOPIC, 10)
        self.pub_status = self.create_publisher(String, TARGETING_STATUS_TOPIC, 10)
        self.pub_cmd = self.create_publisher(Vector3, TURRET_CMD_TOPIC, 10)

        if self.publish_debug:
            self.pub_dbg = self.create_publisher(Image, TARGETING_DEBUG_TOPIC, 10)
            # Kablosuz GCS için JPEG kopya: ham 720p kare ~1900 UDP parçasına
            # bölünüyor ve BEST_EFFORT'ta tek parça kaybı tüm kareyi düşürüyor
            # — WiFi'da ham akış hiç ulaşmıyor. ~50 KB JPEG tek/az parça gider.
            self.pub_dbg_c = self.create_publisher(
                CompressedImage, TARGETING_DEBUG_TOPIC + '/compressed', 10)

        # Nişan, taretin gerçek görüş açısını veren nişan kamerasından
        # (CAMERA_TARET_PROCESSED_TOPIC) alınır — ana sürüş kamerası taretle
        # birlikte dönmediği için onu kullanmak paralaks/hassasiyet hatası
        # yaratırdı (Şartname §6.10: lazer nokta hassasiyeti).
        self.sub = self.create_subscription(
            Image, CAMERA_TARET_PROCESSED_TOPIC,
            self.cb_image, qos_profile_sensor_data)

        # misyon_fsm SHOOT_APPROACH state'i True gönderir, bitince False
        self.create_subscription(Bool, TARGETING_ENABLE_TOPIC, self._on_enable, 10)

        if self.mod_takip:
            self.create_subscription(UInt8, MOD_AKTIF_TOPIC, self._on_mod, 10)

        self.pub_shoot = self.create_publisher(Bool, SHOOT_CMD_TOPIC, 10) \
            if self.auto_fire else None

        self.create_timer(0.05, self._control_loop)  # 20 Hz control loop
        atis = ", auto_fire: hizalanınca atış" if self.auto_fire else ""
        if self.mod_takip:
            self.get_logger().info(
                "TargetingNode başlatıldı — kumanda anahtarını izliyor "
                "(OTONOM'da arar, MANUEL'de durur%s)" % atis)
        elif self.auto_enable:
            self.get_logger().info(
                "TargetingNode başlatıldı — OTONOM (auto_enable%s)" % atis)
        else:
            self.get_logger().info(
                "TargetingNode başlatıldı — STANDBY (misyon_fsm aktifleştirene kadar bekler)")

    def _yolo_yukle(self):
        import os
        yol = self.get_parameter("yolo_model_path").value
        conf = float(self.get_parameter("yolo_conf").value)
        # Paket ve workspace src altında ara (yolo_detection_node ile aynı mantık).
        if not os.path.exists(yol):
            from ament_index_python.packages import get_package_share_directory
            try:
                aday = os.path.join(
                    get_package_share_directory("teknofest_ika"), yol)
                if os.path.exists(aday):
                    yol = aday
            except Exception:
                pass
        if not os.path.exists(yol):
            aday = os.path.expanduser(
                "~/lydia_ws/src/teknofest_ika_yazilim/" + yol)
            if os.path.exists(aday):
                yol = aday
        # `yolo export` ile üretilen .engine ULTRALYTICS formatındadır (başında
        # metadata var); ham TensorRTInferer onu yükleyemez, ultralytics YOLO
        # kendi engine'ini yükleyip TensorRT arka planıyla koşturur (~38 FPS).
        # .engine yoksa aynı arayüzle .pt'ye düşer (yavaş ama çalışır).
        if not HAS_ULTRALYTICS:
            self.get_logger().error(
                "ultralytics yok — nişan YOLO açılamadı, HSV'ye düşülüyor.")
            self.detector = "hsv"
            return
        if yol.endswith(".engine") and os.path.isfile(yol):
            self._ul = UltralyticsYOLO(yol, task="detect")
            self.get_logger().info(f"Nişan YOLO (TensorRT engine): {yol}")
        elif os.path.exists(yol.replace(".engine", ".pt")):
            pt = yol.replace(".engine", ".pt")
            self._ul = UltralyticsYOLO(pt, task="detect")
            self.get_logger().warn(f"Nişan YOLO .engine yok — .pt: {pt}")
        else:
            self.get_logger().error(
                "Nişan YOLO modeli bulunamadı — HSV'ye düşülüyor.")
            self.detector = "hsv"
            return
        self._ul_conf = conf

    def _yolo_hedef_bul(self, img, dbg):
        """En yüksek güvenli hedef_tahtasi kutusunun MERKEZİNİ döndürür."""
        if self._ul is None:
            return None, None
        r = self._ul(img, conf=self._ul_conf, verbose=False)[0]
        en_iyi = None
        for b in r.boxes:
            if YOLO_CLASSES[int(b.cls[0])] != CLASS_HEDEF_TAHTASI:
                continue
            konf = float(b.conf[0])
            if en_iyi is None or konf > en_iyi[0]:
                en_iyi = (konf, b.xyxy[0].tolist())
        if en_iyi is None:
            return None, None
        konf, (x1, y1, x2, y2) = en_iyi
        cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
        if dbg is not None:
            cv2.rectangle(dbg, (int(x1), int(y1)), (int(x2), int(y2)),
                          (0, 255, 0), 3)
            cv2.circle(dbg, (int(cx), int(cy)), 4, (0, 0, 255), -1)
            cv2.putText(dbg, f"hedef {konf:.2f}", (int(x1), max(20, int(y1) - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        return int(round(cx)), int(round(cy))

    def _hsv_hedef_bul(self, img, dbg):
        """Kırmızı halka + Hough çember; en büyük yarıçaplı dairenin merkezi."""
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)
        mask2 = cv2.inRange(hsv, self.hsv_lower2, self.hsv_upper2)
        mask = cv2.bitwise_or(mask1, mask2)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        circles = cv2.HoughCircles(
            mask, cv2.HOUGH_GRADIENT, dp=self.hough_dp,
            minDist=self.hough_min_dist, param1=self.hough_param1,
            param2=self.hough_param2, minRadius=self.hough_min_radius,
            maxRadius=self.hough_max_radius)
        if circles is None:
            return None, None
        circles = np.round(circles[0, :]).astype(int)
        cx, cy, r = max(circles, key=lambda c: c[2])
        if dbg is not None:
            cv2.circle(dbg, (cx, cy), r, (0, 255, 0), 3)
            cv2.circle(dbg, (cx, cy), 2, (0, 0, 255), 3)
        return cx, cy

    def _on_mod(self, msg: UInt8):
        # Kumanda OTONOM'a alınınca hedef arama başlar; MANUEL'e dönünce
        # durur ve taret_rc_koprusu stick kontrolünü devralır. Süren atış
        # darbesi varsa _control_loop başındaki bitirici söndürür.
        otonom = int(msg.data) != MOD_MANUAL
        if otonom == self._enabled:
            return
        self._enabled = otonom
        if otonom:
            self._was_enabled = True
            self.pid_yaw.reset()
            self.pid_pitch.reset()
            self._target_ema = None
            self._aligned_since = None
            self.get_logger().info('Mod OTONOM — hedef aranıyor.')
        else:
            self.get_logger().info('Mod MANUEL — nişan durdu, kontrol RC de.')

    def _on_enable(self, msg):
        if msg.data and not self._enabled:
            self.get_logger().info("TargetingNode: AKTİF — hedef aranıyor")
            self._enabled = True
            self._was_enabled = True
            self.pid_yaw.reset()
            self.pid_pitch.reset()
            self._target_ema = None
            self._aligned_since = None
        elif not msg.data and self._enabled:
            self.get_logger().info("TargetingNode: STANDBY — servo home'a dönüyor")
            self._enabled = False

    def cb_image(self, msg: Image):
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        # Kamera ters monteliyse 180° çevir — hem YOLO tespiti (ters tahtayı
        # zor tanır) hem debug görüntüsü düz olur.
        if self.flip_180:
            img = cv2.rotate(img, cv2.ROTATE_180)
        self._latest_image = img
        # Store ROS 2 timestamp for sync checks (works with both sim and real time)
        self._latest_image_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._latest_image_ros_time = self.get_clock().now()

    def _atis_darbesi(self, now: float) -> None:
        """auto_fire: ALIGNED'da lazeri şartname süresi kadar yakan darbe.

        Kapatma komutu kaybolsa bile taret_rc_koprusu'nun 1.5 s failsafe'i
        söndürür; buradaki süre yalnız normal bitiştir.
        """
        if self._atis_baslangic is not None:
            return                        # darbe zaten sürüyor
        if self._son_atis is not None and (now - self._son_atis) < self.fire_cooldown:
            return
        self._atis_baslangic = now
        self.pub_shoot.publish(Bool(data=True))
        self.get_logger().info("AUTO FIRE — hedef hizalı, lazer tetiklendi.")

    def _atis_darbesi_bitir(self, now: float) -> None:
        if self._atis_baslangic is None:
            return
        if (now - self._atis_baslangic) >= LASER_FIRE_DURATION:
            self.pub_shoot.publish(Bool(data=False))
            self._son_atis = now
            self._atis_baslangic = None

    def _control_loop(self):
        # Süren atış darbesini hedef kaybolsa/devre dışı kalınsa bile bitir.
        if self.auto_fire:
            self._atis_darbesi_bitir(self.get_clock().now().nanoseconds / 1e9)

        # Devre dışıysa: geçişte bir kez home gönder, sonra dur
        if not self._enabled:
            if self._was_enabled:
                self.pub_cmd.publish(Vector3(x=0.0, y=0.0, z=0.0))
                self._publish_status("STANDBY")
                self._was_enabled = False
            return

        if self._latest_image is None or self._latest_image_stamp is None:
            self._publish_status("NO_IMAGE")
            return

        # Use ROS 2 clock consistently (supports both sim_time and real time)
        now_ros = self.get_clock().now()
        now = now_ros.nanoseconds / 1e9
        dt = (now_ros - self._latest_image_ros_time).nanoseconds / 1e9

        # Skip stale images
        if dt > self.image_timeout:
            self._publish_status("STALE_IMAGE")
            return

        # Skip already-processed frame
        if self._last_processed_stamp == self._latest_image_stamp:
            return
        self._last_processed_stamp = self._latest_image_stamp

        img = self._latest_image
        h, w = img.shape[:2]
        cx_img = w / 2.0
        cy_img = h / 2.0

        dbg = img.copy() if self.publish_debug else None

        if self.detector == "yolo":
            target_cx, target_cy = self._yolo_hedef_bul(img, dbg)
        else:
            target_cx, target_cy = self._hsv_hedef_bul(img, dbg)

        if target_cx is not None:
            # Temporal EMA smoothing on target position (anti-vibration)
            if self._target_ema is None:
                self._target_ema = np.array([float(target_cx), float(target_cy)])
            else:
                self._target_ema = (
                    self.target_ema_alpha * np.array([target_cx, target_cy]) +
                    (1.0 - self.target_ema_alpha) * self._target_ema
                )
            tx, ty = self._target_ema

            # Hizalama kontrolü için ham piksel hatası (yön işaretinden bağımsız).
            error_x = tx - cx_img
            error_y = ty - cy_img

            # Kontrol yönü açık işaretle verilir (yaw_sign/pitch_sign) — kamera
            # montajı/flip/köprü PAN_YON birleşimi geri besleme işaretini
            # belirler; yanlışsa taret hedeften kaçar, ilgili ekseni -1 yap.
            yaw_cmd = self.pid_yaw.compute(self.yaw_sign * error_x)
            pitch_cmd = self.pid_pitch.compute(self.pitch_sign * error_y)

            # Güç kısıtı modu: tek seferde TEK servo sürülür (besleme ikisini
            # birden döndüremiyor, komut alan pan aç kalıyordu). Pan bandın
            # dışındaysa önce pan (tilt durur); pan oturunca tilt sürülür.
            if self.tek_eksen:
                if abs(error_x) >= self.align_threshold:
                    pitch_cmd = 0.0
                    self.pid_pitch.reset()
                else:
                    yaw_cmd = 0.0
                    self.pid_yaw.reset()

            # Histerezis: kilitliyken geniş eşiği aşmadıkça kilitli kalır;
            # kilitsizken dar eşiğe girince kilitlenir. Sınırdaki seğirmeyi keser.
            esik = (self.align_threshold * self.unlock_factor
                    if self._locked else self.align_threshold)
            aligned = abs(error_x) < esik and abs(error_y) < esik
            self._locked = aligned

            if aligned:
                # Hizalıyken taret DURUR — hız modunda küçük PID çıktısı bile
                # minimum servo hızına yükselip merkez etrafında seğirtir
                # (limit-cycle), taret bir türlü kilitlenmezdi. Sıfır komut
                # hareketi kesip kilidi sabitler.
                yaw_cmd = 0.0
                pitch_cmd = 0.0
                self.pid_yaw.reset()
                self.pid_pitch.reset()
                if self._aligned_since is None:
                    self._aligned_since = now
                elif (now - self._aligned_since) >= self.fire_lock_dur:
                    self._status = "ALIGNED"
                    if self.auto_fire:
                        self._atis_darbesi(now)
                else:
                    self._status = "LOCKED"
            else:
                self._aligned_since = None
                self._status = "LOCKED"

            self._prev_target_found = True
            self.pub_error.publish(Point(x=float(error_x), y=float(error_y), z=0.0))
            # z=0.0: ateşleme misyon_fsm ShootState tarafından /shoot_command üzerinden yapılır
            self.pub_cmd.publish(Vector3(x=float(yaw_cmd), y=float(pitch_cmd), z=0.0))

            if dbg is not None:
                cv2.putText(dbg, f"ERR: {error_x:.1f}, {error_y:.1f}", (20, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                cv2.putText(dbg, f"CMD: Y{yaw_cmd:.1f} P{pitch_cmd:.1f} [{self._status}]", (20, 80),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        else:
            # Hedef yok — kilit sıfırlanır. Süpürme kapalıysa taret SABİT durur
            # (hedef görüş alanına girene kadar bulunduğu konumu korur).
            if self._prev_target_found:
                self.pid_yaw.reset()
                self.pid_pitch.reset()
                self._target_ema = None
            self._prev_target_found = False
            self._aligned_since = None
            self._locked = False
            if self.sweep:
                self._status = "SEARCHING"
                self._search_yaw += self._search_dir * self._search_speed
                if abs(self._search_yaw) > 30.0:
                    self._search_dir *= -1.0
                self.pub_cmd.publish(Vector3(x=float(self._search_yaw), y=0.0, z=0.0))
            else:
                self._status = "SEARCHING"
                self.pub_cmd.publish(Vector3(x=0.0, y=0.0, z=0.0))  # sabit dur
            self.pub_error.publish(Point(x=0.0, y=0.0, z=0.0))
            if dbg is not None:
                cv2.putText(dbg, "SEARCHING...", (20, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)

        self._publish_status(self._status)

        if dbg is not None:
            self.pub_dbg.publish(self.bridge.cv2_to_imgmsg(dbg, "bgr8"))
            ok, buf = cv2.imencode('.jpg', dbg, [cv2.IMWRITE_JPEG_QUALITY, 40])
            if ok:
                cmsg = CompressedImage()
                cmsg.header.stamp = self.get_clock().now().to_msg()
                cmsg.format = 'jpeg'
                cmsg.data = buf.tobytes()
                self.pub_dbg_c.publish(cmsg)

    def _publish_status(self, status: str):
        self.pub_status.publish(String(data=status))


def main(args=None):
    rclpy.init(args=args)
    node = TargetingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
