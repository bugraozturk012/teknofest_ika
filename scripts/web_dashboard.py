#!/usr/bin/env python3
"""
web_dashboard.py — LYDİA GCS Web Dashboard (Jetson tarafı sunucu)
=================================================================
Eski PyQt dashboard'ın (ika_dashboard.py) yerini alır. Fark mimaridedir:

  ESKİ:  Jetson → (DDS, makineler-arası) → WSL PyQt çizer
         DDS kablosuzda koptuğunda hiçbir şey gelmiyordu.

  YENİ:  Bu sunucu JETSON'DA çalışır, ROS topic'lerine LOKAL abone olur
         (Jetson-içi DDS her zaman sağlam), görüntüleri JPEG'e çevirip
         HTTP ile servis eder. İzleyici herhangi bir tarayıcıdan bakar:
             http://192.168.100.2:8080

  Taşıma katmanları (hepsi düz HTTP/TCP, DDS'ten bağımsız):
    /stream/<ad>  → MJPEG (kamera akışları, JPEG kalite düşük + 480p)
    /veri         → SSE   (sensör değerleri, hazır metin+renk, 5 Hz)
    /sinyaller    → JSON  (sinyal kataloğu: birim + kaynak, bir kez okunur)
    /telemetri    → SSE   (HAM sayılar, 5 Hz)
    /estop  POST  → tarayıcıdan gelen E-STOP'u ROS'a yayınlar

  /veri ile /telemetri arasındaki fark
  ------------------------------------
  /veri biçimlenmiş metin ve renk gönderir ({'t': '+1,2°', 'c': '#ef5350'}).
  Bu, bu dosyanın içindeki SAYFA'yı beslemek için yeterli ama metin grafiğe
  çizilemez, tamponlanamaz, eşiklenemez. /telemetri aynı değerleri HAM sayı
  olarak gönderir; renk ve eşik kararı istemciye kalır.

  Kural: kaynağı olmayan sinyal SAYI DEĞİL null döner. Panoda "—" görünür.
  Uydurulmuş bir sayı ekranda gerçek sanılır — bu dosyada hiçbir sinyal
  tahminden üretilmez, ölçülmeyen şey null'dır.

Kablosuzda ham görüntü asla taşınmaz — her akış Jetson'da sıkıştırılır,
yalnız o akışa bir tarayıcı bağlıyken kodlanır (CPU tasarrufu).

Çalıştırma (Jetson):  ros2 run yok — düz script:
    python3 ~/lydia_ws/src/teknofest_ika_yazilim/scripts/web_dashboard.py
(ROS ortamı .bashrc'den gelir: ROS_DOMAIN_ID=42 + discovery server.)
"""
import glob
import json
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import Bool, UInt8, String, Float32MultiArray, UInt16MultiArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, Imu, Image, CompressedImage, LaserScan
from cv_bridge import CvBridge

from teknofest_ika.otonomi.topics import (
    E_STOP_TOPIC, E_STOP_FORCE_GCS_TOPIC, MOD_AKTIF_TOPIC, FSM_STATE_TOPIC,
    BATTERY_TOPIC, IMU_TOPIC, EKF_ODOM_TOPIC, MISYON_WP_INDEX_TOPIC,
    TARGETING_STATUS_TOPIC, TARGETING_DEBUG_TOPIC, SHOOT_RESULT_TOPIC,
    CAMERA_IMAGE_TOPIC, CAMERA_TARET_TOPIC, YOLO_RAW_DEBUG_TOPIC,
    MAP_IMAGE_TOPIC, DEPTH_IMAGE_TOPIC, SCAN_TOPIC,
    RC_INPUT_TOPIC, MUX_CMD_VEL_TOPIC, ODOM_TOPIC, ENKODER_HAM_TOPIC,
    YOLO_RAW_TOPIC,
)

# vision_msgs kurulu değilse tespit sayacı sessizce kapanır; panonun tamamı
# bu paket yüzünden düşmemeli.
try:
    from vision_msgs.msg import Detection2DArray
except ImportError:
    Detection2DArray = None

# ── Ayarlar ───────────────────────────────────────────────────────────────────
PORT        = 8080
JPEG_KALITE = 35     # düşük = küçük dosya, kablosuz için uygun
MAKS_GENIS  = 480    # kameralar bu genişliğe küçültülür (en-boy korunur)
AKIS_FPS    = 12     # MJPEG akış tavanı (izleme için yeterli)
PANELLER    = {'on', 'yolo', 'nisan', 'harita', 'derinlik', 'lidar'}
# Bu süreden eski kare "sinyal yok" sayılır — donmuş görüntü canlı sanılırsa
# yanlış karar verdirir. SLAM haritası seyrek yayınlandığı için (~5 s) kamera
# eşiğine tabi tutulursa panel sürekli boş görünür, ayrı eşiği var.
KARE_TAZE_S = 3.0
KARE_TAZE_OZEL = {'harita': 30.0}
LIDAR_PANEL_PX  = 400   # radar görüntüsü kenar uzunluğu [px]
LIDAR_MENZIL_M  = 5.0   # panelin kapsadığı yarıçap [m]
# LiDAR montaj yönünü ekran eksenine çevirir: X'te aynalayıp saat yönünde 90°
# döndürünce "yukarı = aracın önü" olur. Yalnız görselleştirme, /scan değişmez.
LIDAR_AYNA_X    = True
LIDAR_DONDUR    = -math.pi / 2.0   # saat yönünde 90°
NISAN_DEBUG_TAZE_S = 1.5   # işaretli nişan görüntüsü bu kadar tazeyse ham yerine o

# Eski dashboard renk paleti (birebir korundu)
BG, PANEL, CARD, LINE = '#181818', '#202020', '#242424', '#2e2e2e'
TEXT, DIM = '#d4d4d4', '#5a5a5a'
GREEN, RED, AMBER, CYAN = '#4caf50', '#ef5350', '#ffa726', '#26c6da'

# ── /battery/status alanlarının anlamı ────────────────────────────────────────
# 0x13 paketi depoda "motor akımı [mA] + batarya [cV]" olarak çözülür, ama
# ARAÇTA KOŞAN firmware (depodakiyle aynı sürüm değil) aynı iki alanı başka
# şey için kullanıyor:
#     bat.current × 10 = gaz voltajı   [V]
#     bat.voltage × 10 = direksiyon açısı [°]
# Bunun sonucu: bu araçta batarya gerilimi ve motor akımı ÖLÇÜLMÜYOR, ölçüm
# alanları dolu. Depodaki firmware bir gün yüklenirse bunu False yap.
ARAC_FIRMWARE_0X13 = True

# ── Sinyal kataloğu — hangi sayının gerçekten kaynağı var ─────────────────────
# İstemci bunu /sinyaller'den bir kez okur ve panoyu buna göre kurar.
# 'kaynak' None ise sinyalin bu araçta karşılığı yoktur; /telemetri onu her
# zaman null döndürür ve panoda "—" görünür. Değer üretebilir hale gelen bir
# sinyalin kaynağı burada doldurulur — tek düzenlenecek yer burasıdır.
SINYAL_KATALOG = [
    # anahtar,        ad,               grup,       birim,    kaynak
    ('gaz_v',         'gaz_v',          'Sürüş',    'V',      '/battery/status current×10'),
    ('gaz_kom',       'gaz_kom',        'Sürüş',    'V',      None),
    ('hiz',           'hiz',            'Sürüş',    'm/s',    '/odometry/filtered (EKF)'),
    ('hiz_komut',     'hiz_komut',      'Sürüş',    'm/s',    '/mux/cmd_vel linear.x'),
    ('steer_aci',     'steer_aci',      'Sürüş',    '°',      '/battery/status voltage×10'),
    ('steer_akim',    'steer_akim',     'Sürüş',    'A',      None),
    ('fren_pwm',      'fren_pwm',       'Sürüş',    'µs',     None),

    ('rc_gaz',        'rc_gaz',         'Kumanda',  'µs',     '/rc_input[0]'),
    ('rc_direksiyon', 'rc_direksiyon',  'Kumanda',  'µs',     '/rc_input[1]'),
    ('rc_mod',        'rc_mod',         'Kumanda',  'µs',     '/rc_input[2]'),
    ('rc_aux',        'rc_aux',         'Kumanda',  'µs',     '/rc_input[3]'),
    ('rc_fren',       'rc_fren',        'Kumanda',  'µs',     None),

    ('v48',           'v48',            'Sensör',   'V',      None),
    ('v12',           'v12',            'Sensör',   'V',      None),
    ('encp',          'encp',           'Sensör',   'sayım',  '/enkoder/ham[0]'),
    ('enc_mesafe',    'enc_mesafe',     'Sensör',   'm',      '/odom pose.x'),
    ('yatis',         'yatis',          'Sensör',   '°',      '/imu/data'),
    ('yunuslama',     'yunuslama',      'Sensör',   '°',      '/imu/data'),

    ('engel',         'engel',          'Algı',     'm',      '/scan en yakın ışın'),
    ('tespit',        'tespit',         'Algı',     'adet',   '/ika/detections'),
    ('fps',           'fps',            'Algı',     'Hz',     'ön kamera kare hızı'),

    ('wp',            'wp_aktif',       'Sistem',   'nokta',  '/misyon/wp_index'),
    ('cpu',           'cpu',            'Sistem',   '%',      '/proc/stat'),
    ('gpu',           'gpu',            'Sistem',   '%',      'sysfs gpu load'),
    ('sicaklik',      'sicaklik',       'Sistem',   '°C',     'sysfs thermal_zone'),
    ('ram',           'ram',            'Sistem',   'MB',     '/proc/meminfo'),
    ('yas',           'yas',            'Sistem',   'sn',     'son ROS mesajından beri'),
]

# Kaynağı olmayan sinyaller — telemetri bunları her zaman null döndürür.
KAYNAKSIZ = frozenset(k for k, _a, _g, _b, kaynak in SINYAL_KATALOG if kaynak is None)

if not ARAC_FIRMWARE_0X13:
    # Depo firmware'i: aynı iki alan gerçek batarya/akım taşır.
    SINYAL_KATALOG = [
        (k, a, g, b,
         '/battery/status voltage' if k == 'v48' else
         '/battery/status current' if k == 'steer_akim' else
         None if k in ('gaz_v', 'steer_aci') else kaynak)
        for k, a, g, b, kaynak in SINYAL_KATALOG
    ]
    KAYNAKSIZ = frozenset(k for k, _a, _g, _b, kk in SINYAL_KATALOG if kk is None)

MOD_ISIMLER = {0: 'MANUAL', 1: 'SEMI AUTO', 2: 'FULL AUTO'}
MOD_RENK    = {'MANUAL': AMBER, 'SEMI AUTO': CYAN, 'FULL AUTO': GREEN}
TARGETING_R = {'SEARCHING': AMBER, 'LOCKED': GREEN, 'ALIGNED': CYAN,
               'STANDBY': DIM, 'NO_IMAGE': RED, 'STALE_IMAGE': RED}


# ── Paylaşılan durum ──────────────────────────────────────────────────────────
class Ortak:
    def __init__(self):
        self.kilit = threading.Lock()
        # panel adı → (bgr numpy, zaman)
        self.kareler = {'on': (None, 0.0), 'yolo': (None, 0.0),
                        'harita': (None, 0.0), 'nisan_ham': (None, 0.0),
                        'nisan_dbg': (None, 0.0), 'derinlik': (None, 0.0),
                        'lidar': (None, 0.0)}
        self.sensor = {
            'e_stop': False, 'mod': 0, 'fsm': '—',
            'batarya': -1.0, 'voltaj': 0.0, 'akim': 0.0,
            'roll': 0.0, 'pitch': 0.0, 'hiz': 0.0, 'wp': 0,
            'targeting': '—', 'shoot': False,
        }
        self.son_ros = 0.0   # en son herhangi bir ROS mesajı zamanı

        # /telemetri için ham sayılar. Başlangıç değeri None — "henüz mesaj
        # gelmedi" ile "değer sıfır" ayrı şeyler; 0.0 ile başlatmak donmuş bir
        # sensörü çalışıyor gibi gösterir.
        self.ham = {k: None for k, _a, _g, _b, _s in SINYAL_KATALOG}
        # Kaynağı olan bir sinyalin en son ne zaman güncellendiği. Mesaj
        # kesilirse değer bayatlar; BAYAT_S sonrası null'a döner.
        self.ham_zaman = {k: 0.0 for k in self.ham}

    def ham_yaz(self, anahtar, deger):
        """Ham sinyal yazar. Kaynağı olmayan anahtar sessizce yok sayılır."""
        if anahtar in KAYNAKSIZ:
            return
        with self.kilit:
            self.ham[anahtar] = deger
            self.ham_zaman[anahtar] = time.time()

    def kare_yaz(self, ad, bgr):
        with self.kilit:
            self.kareler[ad] = (bgr, time.time())

    def kare_oku(self, ad):
        with self.kilit:
            return self.kareler.get(ad, (None, 0.0))


ortak = Ortak()


# ── ROS düğümü ────────────────────────────────────────────────────────────────
class WebDashboardNode(Node):
    def __init__(self):
        super().__init__('web_dashboard')
        self._kopru = CvBridge()
        be = qos_profile_sensor_data

        self.create_subscription(Bool, E_STOP_TOPIC, self._estop, 10)
        self.create_subscription(UInt8, MOD_AKTIF_TOPIC, self._mod, 10)
        self.create_subscription(String, FSM_STATE_TOPIC, self._fsm, 10)
        self.create_subscription(BatteryState, BATTERY_TOPIC, self._bat, 10)
        self.create_subscription(Imu, IMU_TOPIC, self._imu, be)
        self.create_subscription(Odometry, EKF_ODOM_TOPIC, self._odom, be)
        self.create_subscription(UInt8, MISYON_WP_INDEX_TOPIC, self._wp, 10)
        self.create_subscription(String, TARGETING_STATUS_TOPIC, self._targeting, 10)
        self.create_subscription(Bool, SHOOT_RESULT_TOPIC, self._shoot, 10)

        self.create_subscription(Image, CAMERA_IMAGE_TOPIC,
                                 lambda m: self._ham_kare('on', m), be)
        self.create_subscription(Image, YOLO_RAW_DEBUG_TOPIC,
                                 lambda m: self._ham_kare('yolo', m), be)
        self.create_subscription(Image, MAP_IMAGE_TOPIC,
                                 lambda m: self._ham_kare('harita', m), be)
        self.create_subscription(Image, CAMERA_TARET_TOPIC,
                                 lambda m: self._ham_kare('nisan_ham', m), be)
        # Derinlik kamerası — sürücü rgb8 yayınlar (renklendirme kendisinde),
        # cv_bridge bgr8'e çevirir, ek işlem gerekmez.
        self.create_subscription(Image, DEPTH_IMAGE_TOPIC,
                                 lambda m: self._ham_kare('derinlik', m), be)
        self.create_subscription(LaserScan, SCAN_TOPIC, self._tarama, be)
        # Nişan işaretli görüntü — targeting_node JPEG yayını (kablosuz dostu).
        self.create_subscription(CompressedImage, TARGETING_DEBUG_TOPIC + '/compressed',
                                 self._sik_nisan, be)

        # — /telemetri için ek kaynaklar (mevcut panelleri etkilemez) —
        self.create_subscription(Float32MultiArray, RC_INPUT_TOPIC, self._rc, be)
        self.create_subscription(Twist, MUX_CMD_VEL_TOPIC, self._komut, 10)
        self.create_subscription(Odometry, ODOM_TOPIC, self._enk_odom, be)
        self.create_subscription(UInt16MultiArray, ENKODER_HAM_TOPIC, self._enk_ham, be)
        # Ham tespitler /detections/yolo'da (Detection2DArray). /ika/detections
        # yolo_adapter'ın String JSON özeti — farklı tip, farklı topic.
        if Detection2DArray is not None:
            self.create_subscription(Detection2DArray, YOLO_RAW_TOPIC,
                                     self._tespit, be)
        else:
            self.get_logger().warn(
                'vision_msgs yok — tespit sayacı kapalı, /telemetri null döner.')

        self._estop_pub = self.create_publisher(Bool, E_STOP_FORCE_GCS_TOPIC, 10)

        # Ön kamera kare hızı: kare sayılır, saniyede bir orana çevrilir.
        self._fps_sayac = 0
        self._fps_zaman = time.time()
        self.create_timer(1.0, self._fps_hesapla)
        # Jetson kaynakları — sysfs okuması ucuz ama her SSE karesinde değil,
        # saniyede bir örneklenir.
        self._cpu_onceki = None
        self.create_timer(1.0, self._kaynak_ornekle)

    # — görüntü —
    def _ham_kare(self, ad, msg):
        try:
            ortak.kare_yaz(ad, self._kopru.imgmsg_to_cv2(msg, 'bgr8'))
            ortak.son_ros = time.time()
            if ad == 'on':
                self._fps_sayac += 1
        except Exception:
            pass

    def _tarama(self, msg: LaserScan):
        """Ham LaserScan'i tepeden bakışlı radar görüntüsüne çizer.

        SLAM haritası birikimlidir ve gecikmelidir; bu panel o anki lazer
        dönüşünü gösterir, engel aniden girdiğinde ilk oradan görülür.
        """
        try:
            boy = LIDAR_PANEL_PX
            im = np.zeros((boy, boy, 3), np.uint8)
            mrk = boy // 2
            olcek = (boy / 2 - 6) / max(LIDAR_MENZIL_M, 0.1)

            # menzil halkaları (1 m aralık) + eksen çizgileri
            for r in range(1, int(LIDAR_MENZIL_M) + 1):
                cv2.circle(im, (mrk, mrk), int(r * olcek), (38, 38, 38), 1)
            cv2.line(im, (mrk, 0), (mrk, boy), (30, 30, 30), 1)
            cv2.line(im, (0, mrk), (boy, mrk), (30, 30, 30), 1)

            # En yakın geçerli ışın — panelde çizilen halkalardan bağımsız,
            # /telemetri'nin 'engel' sinyali. Menzil filtresi çizimle aynı
            # olmalı ki paneldeki nokta ile sayı çelişmesin.
            gecerli = [r for r in msg.ranges if 0.02 < r < LIDAR_MENZIL_M]
            ortak.ham_yaz('engel', round(min(gecerli), 2) if gecerli else None)

            aci = msg.angle_min
            for mesafe in msg.ranges:
                if 0.02 < mesafe < LIDAR_MENZIL_M:
                    # ROS: +x ileri, +y sola. Ekran: yukarı = ileri.
                    g = (-aci if LIDAR_AYNA_X else aci) + LIDAR_DONDUR
                    x = int(mrk + mesafe * math.sin(g) * olcek)
                    y = int(mrk - mesafe * math.cos(g) * olcek)
                    if 0 <= x < boy and 0 <= y < boy:
                        # tek piksel JPEG sıkıştırmasında kayboluyor
                        cv2.circle(im, (x, y), 1, (90, 255, 90), -1)
                aci += msg.angle_increment

            cv2.circle(im, (mrk, mrk), 2, (60, 160, 255), -1)   # araç
            cv2.putText(im, f'{LIDAR_MENZIL_M:.0f}m', (6, boy - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (120, 120, 120), 1)
            ortak.kare_yaz('lidar', im)
            ortak.son_ros = time.time()
        except Exception:
            pass

    def _sik_nisan(self, msg):
        try:
            img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
            if img is not None:
                ortak.kare_yaz('nisan_dbg', img)
                ortak.son_ros = time.time()
        except Exception:
            pass

    # — sensör —
    def _dokun(self):
        ortak.son_ros = time.time()

    def _estop(self, m):     ortak.sensor['e_stop'] = m.data; self._dokun()
    def _mod(self, m):       ortak.sensor['mod'] = int(m.data); self._dokun()
    def _fsm(self, m):       ortak.sensor['fsm'] = m.data; self._dokun()
    def _wp(self, m):
        ortak.sensor['wp'] = int(m.data)
        ortak.ham_yaz('wp', int(m.data))
        self._dokun()

    def _targeting(self, m): ortak.sensor['targeting'] = m.data; self._dokun()
    def _shoot(self, m):     ortak.sensor['shoot'] = m.data; self._dokun()

    def _bat(self, m):
        ortak.sensor['batarya'] = m.percentage * 100 if m.percentage >= 0 else -1.0
        ortak.sensor['voltaj'] = m.voltage
        ortak.sensor['akim'] = m.current
        # Araç firmware'i bu iki alanı batarya için değil gaz voltajı ve
        # direksiyon açısı için kullanıyor — bkz. ARAC_FIRMWARE_0X13.
        if ARAC_FIRMWARE_0X13:
            ortak.ham_yaz('gaz_v', round(m.current * 10.0, 2))
            ortak.ham_yaz('steer_aci', round(m.voltage * 10.0, 1))
        else:
            ortak.ham_yaz('v48', round(m.voltage, 2))
            ortak.ham_yaz('steer_akim', round(m.current, 2))
        self._dokun()

    def _imu(self, m):
        q = m.orientation
        roll = math.degrees(
            math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x ** 2 + q.y ** 2)))
        sinp = 2 * (q.w * q.y - q.z * q.x)
        pitch = math.degrees(math.asin(max(-1.0, min(1.0, sinp))))
        ortak.sensor['roll'] = roll
        ortak.sensor['pitch'] = pitch
        ortak.ham_yaz('yatis', round(roll, 2))
        ortak.ham_yaz('yunuslama', round(pitch, 2))
        self._dokun()

    def _odom(self, m):
        ortak.sensor['hiz'] = m.twist.twist.linear.x
        ortak.ham_yaz('hiz', round(m.twist.twist.linear.x, 3))
        self._dokun()

    # — /telemetri ham kaynakları —
    def _rc(self, m):
        """RC dizisi kanal SIRASINA göre değil ANLAMINA göre gelir.

        Mega [gaz, direksiyon, mod, aux] gönderir; dizi indeksi kumandadaki
        kanal numarasıyla örtüşmez (mod_yoneticisi._rc_cb ile aynı yorum).
        Bu yüzden burada ch2/ch3/ch4 değil anlam adları kullanılıyor.
        """
        d = m.data
        for i, ad in enumerate(('rc_gaz', 'rc_direksiyon', 'rc_mod', 'rc_aux')):
            if len(d) > i:
                ortak.ham_yaz(ad, round(float(d[i]), 1))
        self._dokun()

    def _komut(self, m):
        ortak.ham_yaz('hiz_komut', round(m.linear.x, 3))
        self._dokun()

    def _enk_odom(self, m):
        # Arka aks tek parça, diferansiyel yok → /odom pose'u yalnız kat edilen
        # mesafeyi taşır, yön bilgisi içermez (bkz. seri_kopru._odometri).
        ortak.ham_yaz('enc_mesafe', round(m.pose.pose.position.x, 3))
        self._dokun()

    def _enk_ham(self, m):
        if len(m.data) > 0:
            ortak.ham_yaz('encp', int(m.data[0]))
        self._dokun()

    def _tespit(self, m):
        ortak.ham_yaz('tespit', len(m.detections))
        self._dokun()

    def _fps_hesapla(self):
        simdi = time.time()
        dt = simdi - self._fps_zaman
        if dt > 0:
            ortak.ham_yaz('fps', round(self._fps_sayac / dt, 1))
        self._fps_sayac = 0
        self._fps_zaman = simdi

    def _cpu_yuzde(self):
        """/proc/stat iki örnek arasındaki meşguliyet oranı."""
        try:
            with open('/proc/stat') as f:
                alan = [float(x) for x in f.readline().split()[1:]]
        except OSError:
            return None
        toplam, bosta = sum(alan), alan[3] + (alan[4] if len(alan) > 4 else 0.0)
        onceki, self._cpu_onceki = self._cpu_onceki, (toplam, bosta)
        if onceki is None:
            return None
        d_top, d_bos = toplam - onceki[0], bosta - onceki[1]
        if d_top <= 0:
            return None
        return round(100.0 * (1.0 - d_bos / d_top), 1)

    def _kaynak_ornekle(self):
        ortak.ham_yaz('cpu', self._cpu_yuzde())
        ortak.ham_yaz('gpu', gpu_yuzde())
        ortak.ham_yaz('sicaklik', sicaklik_c())
        ortak.ham_yaz('ram', ram_mb())

    def gcs_estop(self, aktif):
        self._estop_pub.publish(Bool(data=bool(aktif)))


# ── Jetson kaynakları (sysfs) ─────────────────────────────────────────────────
# Yollar JetPack sürümüne göre değişiyor; hiçbiri bulunamazsa None döner ve
# panoda "—" görünür. Tahmin üretilmez.
def _ilk_sayi(kaliplar):
    for kalip in kaliplar:
        for yol in sorted(glob.glob(kalip)):
            try:
                with open(yol) as f:
                    return float(f.read().strip())
            except (OSError, ValueError):
                continue
    return None


def gpu_yuzde():
    """GPU yükü. sysfs binde (0–1000) verir."""
    ham = _ilk_sayi(['/sys/devices/platform/gpu.0/load',
                     '/sys/devices/gpu.0/load',
                     '/sys/devices/platform/*.gpu/load'])
    return None if ham is None else round(ham / 10.0, 1)


def sicaklik_c():
    """CPU termal bölgesi. Tercihen adı CPU olan bölge, yoksa en sıcak bölge."""
    try:
        bolgeler = sorted(glob.glob('/sys/class/thermal/thermal_zone*'))
    except OSError:
        return None
    en_sicak = None
    for b in bolgeler:
        try:
            with open(os.path.join(b, 'type')) as f:
                tip = f.read().strip().lower()
            with open(os.path.join(b, 'temp')) as f:
                c = float(f.read().strip()) / 1000.0
        except (OSError, ValueError):
            continue
        if not -20.0 < c < 150.0:
            continue
        if 'cpu' in tip:
            return round(c, 1)
        en_sicak = c if en_sicak is None else max(en_sicak, c)
    return None if en_sicak is None else round(en_sicak, 1)


def ram_mb():
    try:
        with open('/proc/meminfo') as f:
            satirlar = dict(
                (p[0].rstrip(':'), float(p[1]))
                for p in (s.split() for s in f) if len(p) >= 2)
    except (OSError, ValueError):
        return None
    top, uygun = satirlar.get('MemTotal'), satirlar.get('MemAvailable')
    if top is None or uygun is None:
        return None
    return round((top - uygun) / 1024.0, 0)


# ── Ham telemetri (yeni pano) ─────────────────────────────────────────────────
# Kaynağı olan ama bu süredir güncellenmemiş sinyal bayat sayılır ve null'a
# döner. Donmuş bir sensörün son değerini göstermeye devam etmek, çalışıyor
# sanılmasına yol açar.
BAYAT_S = 3.0


def sinyal_katalog_json():
    return [{'k': k, 'ad': ad, 'gr': gr, 'bir': bir, 'kaynak': kaynak}
            for k, ad, gr, bir, kaynak in SINYAL_KATALOG]


def telemetri_json():
    s = ortak.sensor
    simdi = time.time()
    bagli = (simdi - ortak.son_ros) < 3.0

    with ortak.kilit:
        deger = dict(ortak.ham)
        zaman = dict(ortak.ham_zaman)

    sinyaller = {}
    for k in deger:
        if k in KAYNAKSIZ or zaman[k] == 0.0 or (simdi - zaman[k]) > BAYAT_S:
            sinyaller[k] = None
        else:
            sinyaller[k] = deger[k]

    # 'yas' bayatlığın kendisini ölçer, bayatlık kuralına tabi değil.
    sinyaller['yas'] = round(simdi - ortak.son_ros, 3) if ortak.son_ros else None

    return {
        'bagli': bagli,
        'estop': bool(s['e_stop']),
        'mod': int(s['mod']),
        'mod_ad': MOD_ISIMLER.get(s['mod'], '?'),
        'fsm': s['fsm'],
        'targeting': s['targeting'],
        'shoot': bool(s['shoot']),
        'sinyal': sinyaller,
    }


# ── Sensör → hazır metin + renk (renk mantığı eski dashboard ile birebir) ─────
def sensor_json():
    s = ortak.sensor
    bagli = (time.time() - ortak.son_ros) < 3.0

    mod = MOD_ISIMLER.get(s['mod'], '?')
    fsm = s['fsm'] or '—'
    fsm_renk = (RED if any(k in fsm for k in ('EMERGENCY', 'ERROR')) else
                AMBER if 'STOP' in fsm else
                DIM if fsm in ('IDLE', '—') else TEXT)
    r, p, h, b, v, a = (s['roll'], s['pitch'], s['hiz'],
                        s['batarya'], s['voltaj'], s['akim'])
    tgt = s['targeting']

    def hc(txt, renk):
        return {'t': txt, 'c': renk}

    return {
        'bagli': bagli,
        'estop': s['e_stop'],
        'mod': hc(mod, MOD_RENK.get(mod, TEXT)),
        'fsm': hc(fsm, fsm_renk),
        'roll': hc(f'{r:+.1f}°', RED if abs(r) > 15 else AMBER if abs(r) > 8 else TEXT),
        'pitch': hc(f'{p:+.1f}°', RED if abs(p) > 15 else AMBER if abs(p) > 8 else TEXT),
        'hiz': hc(f'{h:.2f} m/s', RED if abs(h) > 3.0 else TEXT),
        'wp': hc(str(s['wp']), TEXT),
        'bat': hc(f'{b:.0f}%' if b >= 0 else '—',
                  GREEN if b > 50 else AMBER if b > 20 else RED if b >= 0 else DIM),
        'voltaj': hc(f'{v:.1f} V' if v > 0 else '—',
                     GREEN if v > 29.6 else AMBER if v > 28.0 else RED if v > 0 else DIM),
        'akim': hc(f'{a:.1f} A' if b >= 0 else '—',
                   RED if a > 30 else AMBER if a > 20 else (TEXT if b >= 0 else DIM)),
        'target': hc(tgt, TARGETING_R.get(tgt, TEXT)),
        'lazer': hc('Ateş !' if s['shoot'] else 'Beklemede', RED if s['shoot'] else DIM),
    }


# ── Görüntü kodlama ───────────────────────────────────────────────────────────
def kucult_kodla(bgr):
    h, w = bgr.shape[:2]
    if w > MAKS_GENIS:
        s = MAKS_GENIS / w
        bgr = cv2.resize(bgr, (MAKS_GENIS, max(1, int(h * s))))
    ok, buf = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, JPEG_KALITE])
    return buf.tobytes() if ok else None


def panel_kare(ad):
    """Panel adına göre gösterilecek ham BGR kareyi seçer.

    Bayat kare döndürülmez: kaynak sustuğunda panel donmuş görüntü yerine
    "sinyal yok" göstersin.
    """
    if ad == 'nisan':
        dbg, tdbg = ortak.kare_oku('nisan_dbg')
        if dbg is not None and (time.time() - tdbg) < NISAN_DEBUG_TAZE_S:
            return dbg
        ham, tham = ortak.kare_oku('nisan_ham')
        return ham if (ham is not None and time.time() - tham < KARE_TAZE_S) else None
    kare, t = ortak.kare_oku(ad)
    esik = KARE_TAZE_OZEL.get(ad, KARE_TAZE_S)
    return kare if (kare is not None and time.time() - t < esik) else None


# ── HTML sayfası ──────────────────────────────────────────────────────────────
SAYFA = """<!doctype html><html lang=tr><head><meta charset=utf-8>
<meta name=viewport content="width=device-width,initial-scale=1">
<title>LYDİA İKA — GCS</title><style>
*{box-sizing:border-box;margin:0;padding:0}
body{background:%(BG)s;color:%(TEXT)s;font:13px 'Segoe UI',system-ui,sans-serif}
#bar{height:34px;background:%(PANEL)s;border:1px solid %(LINE)s;border-radius:6px;
 margin:6px;display:flex;align-items:center;padding:0 12px;gap:10px}
#bar b{font-size:14px}#bar .s{color:%(DIM)s;font-size:12px}
#bar .sag{margin-left:auto;display:flex;gap:12px;align-items:center}
#bag{font-size:12px;color:%(DIM)s}#saat{font-size:13px;color:%(DIM)s;font-family:monospace}
#ana{display:flex;gap:6px;margin:0 6px 6px}
#sol{width:178px;display:flex;flex-direction:column;gap:6px}
/* minmax(0,1fr): hücre tabanı içerikten bağımsız olsun. Düz 1fr min-content'i
   taban alır, SLAM haritası büyüdükçe panel boyu da oynar. */
#orta{flex:1;display:grid;grid-template-columns:repeat(3,minmax(0,1fr));grid-template-rows:repeat(2,minmax(0,1fr));gap:6px;min-height:0}
.vid{min-width:0;min-height:0;overflow:hidden}
#sag{width:200px;display:flex;flex-direction:column;gap:6px}
.kart{background:%(CARD)s;border:1px solid %(LINE)s;border-radius:6px;padding:8px 10px}
.kart .b{font-size:11px;color:%(DIM)s;margin-bottom:3px}
.kart .v{font-size:15px;font-weight:bold;word-wrap:break-word}
.vid{background:#111;border:1px solid %(LINE)s;border-radius:6px;overflow:hidden;
 display:flex;flex-direction:column;min-height:0}
.vid .h{height:20px;background:%(LINE)s;display:flex;align-items:center;padding:0 8px;
 font-size:11px;color:%(DIM)s}
.vid .g{flex:1;min-height:0;min-width:0;position:relative;overflow:hidden}
/* Mutlak konum: kare boyutu değiştiğinde (SLAM haritası büyürken olduğu gibi)
   hücreyi itip paneli büyütmez. */
.vid img{position:absolute;top:0;left:0;width:100%%;height:100%%;object-fit:contain}
/* SLAM haritası OccupancyGrid çözünürlüğünde gelir (~230px) — panele
   büyütülürken hücre sınırları keskin kalsın, bulanıklaşmasın. */
#cam_harita{image-rendering:pixelated}
.vid .yok{position:absolute;top:50%%;left:50%%;transform:translate(-50%%,-50%%);color:%(DIM)s;font-size:12px}
button{border:none;border-radius:4px;cursor:pointer;font-family:inherit}
#estop{background:%(RED)s;color:#fff;font-weight:bold;font-size:14px;padding:8px;width:100%%}
#estop:hover{background:#d32f2f}
#kaldir{background:%(LINE)s;color:%(TEXT)s;font-size:11px;padding:4px;width:100%%;margin-top:4px}
#kaldir:hover{background:#383838}
.srow{display:flex;justify-content:space-between;align-items:center;
 padding:6px 10px;border-bottom:1px solid %(LINE)s}
.srow:last-child{border-bottom:none}
.srow .k{color:%(DIM)s;font-size:12px}.srow .d{font-weight:bold;font-family:monospace}
#surekart .v{font-family:monospace;font-size:20px}
.sbtn{background:%(LINE)s;color:%(TEXT)s;font-size:11px;padding:4px 8px;margin-top:4px}
</style></head><body>
<div id=bar><b>LYDİA İKA</b><span class=s>|</span>
 <span class=s>MAGNESIA · TEKNOFEST 2026</span>
 <div class=sag><span id=bag>bağlantı yok</span><span id=saat>--:--:--</span></div></div>
<div id=ana>
 <div id=sol>
  <div class=kart><div class=b>ACİL DURDURMA</div>
   <div class=v id=estopv>—</div>
   <button id=estop onclick=estopBas()>ACİL DURDUR</button>
   <button id=kaldir onclick=estopKaldir()>E-STOP Kaldır (GCS)</button></div>
  <div class=kart><div class=b>Mod</div><div class=v id=modv>—</div></div>
  <div class=kart><div class=b>Parkur Aşaması</div><div class=v id=fsmv>—</div></div>
  <div class=kart id=surekart><div class=b>Süre</div><div class=v id=surev>00:00.0</div>
   <button class=sbtn onclick=sureTog()>Başlat/Durdur</button>
   <button class=sbtn onclick=sureSif()>Sıfırla</button></div>
 </div>
 <div id=orta>
  <div class=vid><div class=h>Ön Kamera</div><div class=g>
   <span class=yok>sinyal yok</span><img id=cam_on></div></div>
  <div class=vid><div class=h>YOLO Debug</div><div class=g>
   <span class=yok>sinyal yok</span><img id=cam_yolo></div></div>
  <div class=vid><div class=h>Nişan Kamera</div><div class=g>
   <span class=yok>sinyal yok</span><img id=cam_nisan></div></div>
  <div class=vid><div class=h>SLAM Haritası</div><div class=g>
   <span class=yok>sinyal yok</span><img id=cam_harita></div></div>
  <div class=vid><div class=h>Derinlik Kamerası</div><div class=g>
   <span class=yok>sinyal yok</span><img id=cam_derinlik></div></div>
  <div class=vid><div class=h>LiDAR (anlık)</div><div class=g>
   <span class=yok>sinyal yok</span><img id=cam_lidar></div></div>
 </div>
 <div id=sag>
  <div class=kart style=padding:0>
   <div class=srow><span class=k>Batarya</span><span class=d id=s_bat>—</span></div>
   <div class=srow><span class=k>Voltaj</span><span class=d id=s_voltaj>—</span></div>
   <div class=srow><span class=k>Akım</span><span class=d id=s_akim>—</span></div>
   <div class=srow><span class=k>Roll</span><span class=d id=s_roll>—</span></div>
   <div class=srow><span class=k>Pitch</span><span class=d id=s_pitch>—</span></div>
   <div class=srow><span class=k>Hız</span><span class=d id=s_hiz>—</span></div>
   <div class=srow><span class=k>Waypoint</span><span class=d id=s_wp>—</span></div>
   <div class=srow><span class=k>Nişan</span><span class=d id=s_target>—</span></div>
   <div class=srow><span class=k>Lazer</span><span class=d id=s_lazer>—</span></div>
  </div>
 </div>
</div>
<script>
// Kamera panelleri — her kare ayrı istek. MJPEG bağlantıyı süresiz açık tutar,
// 6 panel + SSE tarayıcının 6 eşzamanlı bağlantı sınırını doldurur.
for(const [id,ad] of [['cam_on','on'],['cam_yolo','yolo'],
                      ['cam_nisan','nisan'],['cam_harita','harita'],
                      ['cam_derinlik','derinlik'],['cam_lidar','lidar']]){
  const im=document.getElementById(id), yok=im.previousElementSibling;
  let bekliyor=false;
  const cek=()=>{
    if(bekliyor) return;              // önceki kare inmeden yenisini isteme
    bekliyor=true;
    const y=new Image();
    y.onload=()=>{im.src=y.src; yok.style.display='none'; bekliyor=false};
    y.onerror=()=>{yok.style.display='block'; bekliyor=false};
    y.src='/kare/'+ad+'?t='+Date.now();
  };
  cek();
  setInterval(cek, 100);              // ~10 fps tavan
}
// Sensör akışı — SSE
const es=new EventSource('/veri');
es.onmessage=e=>{
  const d=JSON.parse(e.data);
  const bag=document.getElementById('bag');
  bag.textContent=d.bagli?'bağlı':'bağlantı kesildi';
  bag.style.color=d.bagli?'%(GREEN)s':'%(RED)s';
  document.getElementById('estopv').textContent=d.estop?'AKTİF':'Normal';
  document.getElementById('estopv').style.color=d.estop?'%(RED)s':'%(GREEN)s';
  set('modv',d.mod);set('fsmv',d.fsm);
  set('s_bat',d.bat);set('s_voltaj',d.voltaj);set('s_akim',d.akim);
  set('s_roll',d.roll);set('s_pitch',d.pitch);set('s_hiz',d.hiz);
  set('s_wp',d.wp);set('s_target',d.target);set('s_lazer',d.lazer);
};
function set(id,o){const el=document.getElementById(id);if(!el||!o)return;
  el.textContent=o.t;el.style.color=o.c;}
// E-STOP
function estopBas(){fetch('/estop',{method:'POST',body:JSON.stringify({aktif:true})});}
function estopKaldir(){if(confirm('E-STOP GCS kaynağı kaldırılsın mı?\\n(Diğer kaynaklar hâlâ aktifse durdurma sürer.)'))
  fetch('/estop',{method:'POST',body:JSON.stringify({aktif:false})});}
// Saat
setInterval(()=>{document.getElementById('saat').textContent=
  new Date().toLocaleTimeString('tr-TR');},1000);
// Süre kronometresi (tarayıcı tarafı)
let sT=0,sBas=0,sInt=null;
function sureTog(){if(sInt){clearInterval(sInt);sInt=null;sT+=Date.now()-sBas;}
  else{sBas=Date.now();sInt=setInterval(sureCiz,100);}}
function sureSif(){clearInterval(sInt);sInt=null;sT=0;sureCiz();}
function sureCiz(){let ms=sT+(sInt?Date.now()-sBas:0);
  let s=Math.floor(ms/1000),m=Math.floor(s/60);
  document.getElementById('surev').textContent=
    String(m).padStart(2,'0')+':'+String(s%%60).padStart(2,'0')+'.'+Math.floor(ms%%1000/100);}
</script></body></html>""" % dict(
    BG=BG, PANEL=PANEL, CARD=CARD, LINE=LINE, TEXT=TEXT, DIM=DIM,
    GREEN=GREEN, RED=RED)


# ── Pano ──────────────────────────────────────────────────────────────────────
# Tasarım dili: Apple sistem grileri. Renk YALNIZ durum bildirir (yeşil/turuncu/
# kırmızı) ve her zaman metin etiketiyle birlikte gelir; seçim ve vurgu renkle
# değil dolgu ve ağırlıkla yapılır. Sayılar monospace + tabular-nums — teknik
# veride sütun hizası şart.
#
# Bu sayfa /sinyaller (katalog, bir kez) + /telemetri (HAM sayı, SSE 5 Hz)
# tüketir, /veri'ye hiç bağlanmaz. Eşik ve renk kararı burada verilir; sunucu
# yalnız sayı gönderir. Eski pano /eski adresinde duruyor.
#
# Python % biçimlendirmesi BİLEREK kullanılmadı: CSS yüzde değerleriyle dolu,
# %% kaçışı bu boyutta hataya davetiye. Renkler CSS değişkenlerinde.
SAYFA_YENI = """<!doctype html><html lang=tr><head><meta charset=utf-8>
<meta name=viewport content="width=device-width,initial-scale=1">
<title>LYDİA İKA — Pano</title><style>
:root{
  color-scheme:dark light;
  --metin:-apple-system,BlinkMacSystemFont,"SF Pro Text","Segoe UI Variable Text",
          "Segoe UI",system-ui,sans-serif;
  --sayi:ui-monospace,"SF Mono","Cascadia Mono",Menlo,Consolas,
         "DejaVu Sans Mono",monospace;
  /* Sahada koyu kullanılıyor; taban koyu, açık tema OS tercihine bırakıldı. */
  --zemin:#000000; --kat1:#1C1C1E; --kat2:#2C2C2E; --kat3:#3A3A3C;
  --ayrac:rgba(84,84,88,.62); --ayrac2:rgba(84,84,88,.32);
  --y1:rgba(255,255,255,.96); --y2:rgba(235,235,245,.62);
  --y3:rgba(235,235,245,.30);
  --dolgu:rgba(118,118,128,.24); --dolgu2:rgba(118,118,128,.38);
  --vurgu:#F2F2F7; --vurgu-yazi:#000000;
  --yesil:#30D158; --turuncu:#FF9F0A; --kirmizi:#FF453A; --gri:#8E8E93;
  --yesil-z:rgba(48,209,88,.15); --turuncu-z:rgba(255,159,10,.15);
  --kirmizi-z:rgba(255,69,58,.15);
  --golge:0 0 0 .5px rgba(255,255,255,.06), 0 4px 16px -8px rgba(0,0,0,.9);
  --bulanik:saturate(180%) blur(20px);
  --ustY:52px;
}
@media (prefers-color-scheme:light){
  :root{
    --zemin:#F2F2F7; --kat1:#FFFFFF; --kat2:#E9E9EF; --kat3:#DCDCE2;
    --ayrac:rgba(60,60,67,.28); --ayrac2:rgba(60,60,67,.12);
    --y1:rgba(0,0,0,.92); --y2:rgba(60,60,67,.60); --y3:rgba(60,60,67,.30);
    --dolgu:rgba(118,118,128,.12); --dolgu2:rgba(118,118,128,.20);
    --vurgu:#1C1C1E; --vurgu-yazi:#F2F2F7;
    --yesil:#248A3D; --turuncu:#B25000; --kirmizi:#D70015;
    --yesil-z:rgba(36,138,61,.12); --turuncu-z:rgba(178,80,0,.12);
    --kirmizi-z:rgba(215,0,21,.10);
    --golge:0 0 0 .5px rgba(0,0,0,.04), 0 4px 14px -8px rgba(0,0,0,.28);
  }
}
*{box-sizing:border-box;margin:0;padding:0}
html,body{height:100%}
body{background:var(--zemin);color:var(--y1);font:14px var(--metin);
     overflow:hidden}
:focus-visible{outline:2px solid var(--y1);outline-offset:2px;border-radius:5px}
@media (prefers-reduced-motion:reduce){*{transition:none!important;animation:none!important}}
.sayi{font-family:var(--sayi);font-variant-numeric:tabular-nums;
      font-feature-settings:"tnum" 1}
.mikro{font-size:10px;letter-spacing:.06em;text-transform:uppercase;
       font-weight:590;color:var(--y3)}

/* ═════════ ÜST ŞERİT ═════════ */
header{display:flex;align-items:center;gap:10px;flex-wrap:wrap;
       padding:9px 14px;height:var(--ustY);
       background:color-mix(in srgb,var(--zemin) 76%,transparent);
       -webkit-backdrop-filter:var(--bulanik);backdrop-filter:var(--bulanik);
       border-bottom:.5px solid var(--ayrac);position:relative;z-index:60}
h1{margin:0 4px 0 0;font-size:15px;font-weight:600;letter-spacing:-.02em;
   white-space:nowrap}
h1 b{font-weight:400;color:var(--y2)}
.rz{display:inline-flex;align-items:center;gap:5px;padding:3.5px 10px 3.5px 8px;
    border-radius:980px;font-size:10px;font-weight:600;letter-spacing:.045em;
    text-transform:uppercase;white-space:nowrap;color:var(--y3);
    background:var(--dolgu);transition:color .2s,background .2s}
.rz::before{content:"";width:6px;height:6px;border-radius:50%;flex:none;
            background:currentColor}
.rz.g{color:var(--yesil);background:var(--yesil-z)}
.rz.t{color:var(--turuncu);background:var(--turuncu-z)}
.rz.k{color:var(--kirmizi);background:var(--kirmizi-z)}
.rz.k::before{animation:nabiz 1.5s ease-in-out infinite}
@keyframes nabiz{0%,100%{opacity:1}50%{opacity:.3}}
.anahtar{display:flex;background:var(--dolgu);border-radius:9px;padding:2px;
         gap:2px;margin-left:6px}
.anahtar button{font:inherit;font-size:12.5px;font-weight:540;padding:5px 13px;
     border:none;border-radius:7px;background:transparent;color:var(--y2);
     cursor:pointer;transition:background .18s,color .18s}
.anahtar button:hover{color:var(--y1)}
.anahtar button[aria-pressed="true"]{background:var(--kat1);color:var(--y1);
     box-shadow:var(--golge);font-weight:590}
.sagUst{margin-left:auto;display:flex;align-items:center;gap:8px}
.hiz{font-family:var(--sayi);font-size:11px;color:var(--y3);
     font-variant-numeric:tabular-nums;white-space:nowrap}
.estopD{font:inherit;font-size:11px;font-weight:700;letter-spacing:.06em;
     padding:7px 13px;border:none;border-radius:8px;cursor:pointer;
     background:var(--kirmizi);color:#fff}
.estopD:hover{filter:brightness(1.12)}
.estopD.kaldir{background:var(--dolgu);color:var(--y2);font-weight:600;
     letter-spacing:.04em}

/* ═════════ GÖRÜNÜM KABI ═════════ */
#sahne{position:absolute;inset:0;top:var(--ustY);overflow:hidden}
.gorunum{position:absolute;inset:0;display:none;overflow:auto;padding:10px;
         gap:10px}
.gorunum.aktif{display:grid}

/* ═════════ KART ═════════ */
.wj{background:var(--kat1);border-radius:14px;box-shadow:var(--golge);
    overflow:hidden;display:flex;flex-direction:column;min-width:0;min-height:0}
.wjUst{display:flex;align-items:center;gap:7px;padding:9px 12px 7px;flex:none}
.wjUst h3{margin:0;font-size:10.5px;font-weight:590;letter-spacing:.05em;
     text-transform:uppercase;color:var(--y2);flex:1;white-space:nowrap;
     overflow:hidden;text-overflow:ellipsis}
.wjUst .ek{font-family:var(--sayi);font-size:10px;color:var(--y3);
     font-variant-numeric:tabular-nums;white-space:nowrap}
.wjGov{padding:0 12px 12px;flex:1;min-width:0;min-height:0;
     display:flex;flex-direction:column}
.dev{font-family:var(--sayi);font-variant-numeric:tabular-nums;
     font-size:29px;font-weight:500;letter-spacing:-.04em;line-height:1.05;
     color:var(--y1)}
.dev em{font-style:normal;font-size:12px;font-family:var(--metin);
     color:var(--y3);margin-left:3px;letter-spacing:0}
.dev.g{color:var(--yesil)} .dev.t{color:var(--turuncu)} .dev.k{color:var(--kirmizi)}
.dev.yok{color:var(--y3)}
.altbil{font-size:11.5px;color:var(--y3);margin-top:3px}
.wjSat{display:flex;justify-content:space-between;align-items:baseline;
     gap:12px;padding:6px 0;font-size:12.5px}
.wjSat+.wjSat{border-top:.5px solid var(--ayrac2)}
.wjSat span:first-child{color:var(--y2)}
.wjSat span:last-child{font-family:var(--sayi);font-variant-numeric:tabular-nums}
.wjSat .t{color:var(--turuncu)} .wjSat .k{color:var(--kirmizi)}
.wjSat .yok{color:var(--y3)}
.metinD{font-size:19px;font-weight:600;letter-spacing:-.02em;line-height:1.15;
     word-break:break-word}

/* ═════════ KAMERA ═════════ */
.kamAlan{position:relative;flex:1;min-height:0;min-width:0;overflow:hidden;
    border-radius:9px;background:#080C12;display:grid;place-items:center}
.kamAlan img{position:absolute;inset:0;width:100%;height:100%;
    object-fit:contain;display:block}
.kamAlan.pikselli img{image-rendering:pixelated}
.kamYok{font-size:11.5px;color:var(--y3);z-index:2}
.kamAlan.canli .kamYok{display:none}

/* ═════════ SÜRÜŞ DÜZENİ ═════════ */
#gSurus{grid-template-columns:220px minmax(0,1fr) 268px;
        grid-template-rows:minmax(0,1fr)}
#surLidar{flex:none}
#surLidar .kamAlan{aspect-ratio:1/1}
#surSol,#surSag{display:flex;flex-direction:column;gap:10px;min-height:0;
        overflow:auto}
#surOrta{display:flex;flex-direction:column;gap:10px;min-height:0}

/* ═════════ OTONOM DÜZENİ ═════════ */
#gOtonom{grid-template-columns:minmax(0,1fr) 240px;grid-template-rows:auto minmax(0,1fr)}
#otoUst{grid-column:1/-1;display:grid;grid-template-columns:repeat(4,minmax(0,1fr));
        gap:10px}
#otoKam{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));
        grid-template-rows:repeat(2,minmax(0,1fr));gap:10px;min-height:0}
#otoSag{display:flex;flex-direction:column;gap:10px;min-height:0;overflow:auto}

/* ═════════ KAYIT DÜZENİ ═════════ */
#gKayit{grid-template-rows:auto minmax(0,1fr);align-content:start}
#kayArac{display:flex;align-items:center;gap:7px;flex-wrap:wrap}
.cip{font:inherit;font-size:12px;font-weight:510;padding:5px 11px;
     border-radius:7px;border:none;background:var(--dolgu);color:var(--y2);
     cursor:pointer;white-space:nowrap;transition:background .15s,color .15s}
.cip:hover{color:var(--y1)}
.cip[aria-pressed="true"]{background:var(--kat1);color:var(--y1);
     box-shadow:var(--golge);font-weight:580}
#kayGovde{overflow:auto;min-height:0}
.kayGrup{margin-bottom:6px}
.kayGrupBas{padding:10px 4px 5px}
table.kay{width:100%;border-collapse:collapse;background:var(--kat1);
     border-radius:14px;overflow:hidden;box-shadow:var(--golge)}
table.kay{table-layout:fixed}
table.kay td{padding:5px 12px;border-top:.5px solid var(--ayrac2);
     vertical-align:middle}
table.kay tr:first-child td{border-top:none}
td.kAd{font-size:12.5px;color:var(--y1);white-space:nowrap;width:190px;
     overflow:hidden;text-overflow:ellipsis}
td.kDeg{font-family:var(--sayi);font-variant-numeric:tabular-nums;
     font-size:13px;text-align:right;white-space:nowrap;width:130px}
td.kDeg em{font-style:normal;font-size:10.5px;color:var(--y3);margin-left:4px}
/* Eğilim sütunu esnek: kalan genişliği o alır. Tuval arka belleği sabit
   (1200×44) ve CSS ile küçültülüyor — büyütülmediği için bulanıklaşmaz. */
td.kCiz{padding:3px 12px}
td.kCiz canvas{display:block;width:100%;height:22px}
td.kKay{font-family:var(--sayi);font-size:10.5px;color:var(--y3);
     text-align:right;white-space:nowrap;overflow:hidden;text-overflow:ellipsis;
     width:230px}
tr.kaynaksiz td.kAd{color:var(--y3)}
tr.kaynaksiz td.kDeg{color:var(--y3)}
.kDeg.t{color:var(--turuncu)} .kDeg.k{color:var(--kirmizi)}

@media (max-width:1100px){
  #gSurus{grid-template-columns:minmax(0,1fr);grid-auto-rows:min-content}
  #gOtonom{grid-template-columns:minmax(0,1fr)}
  #otoUst{grid-template-columns:repeat(2,minmax(0,1fr))}
}
</style></head><body>

<header>
  <h1>LYDİA <b>İKA</b></h1>
  <span class="rz" id=rzBag>bağlantı yok</span>
  <span class="rz" id=rzEstop>e-stop</span>
  <span class="rz" id=rzMod>mod</span>
  <div class="anahtar" role=group aria-label="Görünüm">
    <button type=button data-gor=gSurus aria-pressed=true>Sürüş</button>
    <button type=button data-gor=gOtonom aria-pressed=false>Otonom</button>
    <button type=button data-gor=gKayit aria-pressed=false>Kayıt</button>
  </div>
  <div class=sagUst>
    <span class=hiz id=sure>00:00.0</span>
    <button class=cip type=button id=sureD>Kronometre</button>
    <span class=hiz id=saat>--:--:--</span>
    <button class=estopD type=button id=estopBas>ACİL DURDUR</button>
    <button class="estopD kaldir" type=button id=estopKaldir>Kaldır</button>
  </div>
</header>

<div id=sahne>

  <!-- ══════ SÜRÜŞ ══════ -->
  <section class="gorunum aktif" id=gSurus>
    <div id=surSol>
      <div class=wj><div class=wjUst><h3>Hız</h3><span class=ek>/odometry/filtered</span></div>
        <div class=wjGov><div class="dev sayi" data-dev=hiz>—<em>m/s</em></div>
        <div class=altbil>komut <span class="sayi" data-inline=hiz_komut>—</span> m/s</div></div></div>
      <div class=wj><div class=wjUst><h3>Gaz</h3><span class=ek>DAC</span></div>
        <div class=wjGov><div class="dev sayi" data-dev=gaz_v>—<em>V</em></div>
        <div class=altbil>rölanti 0,90 · tam gaz 2,84</div></div></div>
      <div class=wj><div class=wjUst><h3>Direksiyon</h3><span class=ek>ölü hesap</span></div>
        <div class=wjGov><div class="dev sayi" data-dev=steer_aci>—<em>°</em></div></div></div>
      <div class=wj><div class=wjUst><h3>Enkoder</h3><span class=ek>/odom</span></div>
        <div class=wjGov><div class="dev sayi" data-dev=enc_mesafe>—<em>m</em></div>
        <div class=altbil>ham <span class="sayi" data-inline=encp>—</span> sayım</div></div></div>
    </div>
    <div id=surOrta>
      <div class=wj style="flex:1"><div class=wjUst><h3>Ön Kamera</h3><span class=ek data-kamfps=on>—</span></div>
        <div class=wjGov><div class="kamAlan" data-kam=on><span class=kamYok>sinyal yok</span><img alt=""></div></div></div>
    </div>
    <div id=surSag>
      <!-- LiDAR taraması kare; geniş panelde genişliğin yarısı boşa gidiyordu,
           dar sütunda kendi oranında oturuyor. -->
      <div class=wj id=surLidar><div class=wjUst><h3>LiDAR</h3><span class=ek>5 m yarıçap</span></div>
        <div class=wjGov><div class="kamAlan" data-kam=lidar><span class=kamYok>sinyal yok</span><img alt=""></div></div></div>
      <div class=wj><div class=wjUst><h3>Kumanda</h3><span class=ek>/rc_input</span></div>
        <div class=wjGov>
          <div class=wjSat><span>Gaz</span><span data-sat=rc_gaz>—</span></div>
          <div class=wjSat><span>Direksiyon</span><span data-sat=rc_direksiyon>—</span></div>
          <div class=wjSat><span>Mod</span><span data-sat=rc_mod>—</span></div>
          <div class=wjSat><span>AUX</span><span data-sat=rc_aux>—</span></div>
        </div></div>
      <div class=wj><div class=wjUst><h3>Gövde</h3><span class=ek>/imu/data</span></div>
        <div class=wjGov>
          <div class=wjSat><span>Yatış</span><span data-sat=yatis>—</span></div>
          <div class=wjSat><span>Yunuslama</span><span data-sat=yunuslama>—</span></div>
        </div></div>
      <div class=wj><div class=wjUst><h3>Jetson</h3><span class=ek>sysfs</span></div>
        <div class=wjGov>
          <div class=wjSat><span>CPU</span><span data-sat=cpu>—</span></div>
          <div class=wjSat><span>GPU</span><span data-sat=gpu>—</span></div>
          <div class=wjSat><span>Sıcaklık</span><span data-sat=sicaklik>—</span></div>
          <div class=wjSat><span>RAM</span><span data-sat=ram>—</span></div>
          <div class=wjSat><span>Veri yaşı</span><span data-sat=yas>—</span></div>
        </div></div>
    </div>
  </section>

  <!-- ══════ OTONOM ══════ -->
  <section class=gorunum id=gOtonom>
    <div id=otoUst>
      <div class=wj><div class=wjUst><h3>Parkur Aşaması</h3><span class=ek>/fsm_state</span></div>
        <div class=wjGov><div class=metinD id=oFsm>—</div></div></div>
      <div class=wj><div class=wjUst><h3>Waypoint</h3><span class=ek>/misyon/wp_index</span></div>
        <div class=wjGov><div class="dev sayi" data-dev=wp>—</div></div></div>
      <div class=wj><div class=wjUst><h3>Nişan</h3><span class=ek>/targeting/status</span></div>
        <div class=wjGov><div class=metinD id=oTgt>—</div></div></div>
      <div class=wj><div class=wjUst><h3>Lazer</h3><span class=ek>/shoot/result</span></div>
        <div class=wjGov><div class=metinD id=oLazer>—</div></div></div>
    </div>
    <div id=otoKam>
      <div class=wj><div class=wjUst><h3>YOLO</h3><span class=ek data-kamfps=yolo>—</span></div>
        <div class=wjGov><div class="kamAlan" data-kam=yolo><span class=kamYok>sinyal yok</span><img alt=""></div></div></div>
      <div class=wj><div class=wjUst><h3>SLAM Haritası</h3><span class=ek data-kamfps=harita>—</span></div>
        <div class=wjGov><div class="kamAlan pikselli" data-kam=harita><span class=kamYok>sinyal yok</span><img alt=""></div></div></div>
      <div class=wj><div class=wjUst><h3>Derinlik</h3><span class=ek>OS30A · 0,02–2,5 m</span></div>
        <div class=wjGov><div class="kamAlan" data-kam=derinlik><span class=kamYok>sinyal yok</span><img alt=""></div></div></div>
      <div class=wj><div class=wjUst><h3>Nişan Kamerası</h3><span class=ek data-kamfps=nisan>—</span></div>
        <div class=wjGov><div class="kamAlan" data-kam=nisan><span class=kamYok>sinyal yok</span><img alt=""></div></div></div>
    </div>
    <div id=otoSag>
      <div class=wj><div class=wjUst><h3>Algı</h3><span class=ek>LiDAR + YOLO</span></div>
        <div class=wjGov>
          <div class=wjSat><span>En yakın engel</span><span data-sat=engel>—</span></div>
          <div class=wjSat><span>Tespit</span><span data-sat=tespit>—</span></div>
          <div class=wjSat><span>Kamera</span><span data-sat=fps>—</span></div>
        </div></div>
      <div class=wj><div class=wjUst><h3>Sürüş</h3><span class=ek>otonom komut</span></div>
        <div class=wjGov>
          <div class=wjSat><span>Hız</span><span data-sat=hiz>—</span></div>
          <div class=wjSat><span>Komut</span><span data-sat=hiz_komut>—</span></div>
          <div class=wjSat><span>Direksiyon</span><span data-sat=steer_aci>—</span></div>
          <div class=wjSat><span>Mesafe</span><span data-sat=enc_mesafe>—</span></div>
        </div></div>
      <div class=wj><div class=wjUst><h3>Not</h3></div>
        <div class=wjGov><div class=altbil>Tarayıcıdan araca yazan tek komut
          E-STOP'tur. Otonoma geçiş kumandadaki mod anahtarından yapılır.</div></div></div>
    </div>
  </section>

  <!-- ══════ KAYIT ══════ -->
  <section class=gorunum id=gKayit>
    <div id=kayArac>
      <button class=cip type=button id=kayHepsi aria-pressed=true>Tümü</button>
      <span class=mikro id=kaySayim></span>
    </div>
    <div id=kayGovde></div>
  </section>

</div>

<script>
"use strict";
// ── Biçimlendirme ──────────────────────────────────────────────────────────
// Ondalık basamak birime göre: µs/sayım tam sayı, gerilim iki hane vb.
const OND={'m/s':2,'V':2,'\\u00b0':1,'m':2,'\\u00b5s':0,'sayım':0,'adet':0,
           'Hz':1,'%':0,'\\u00b0C':1,'MB':0,'nokta':0,'sn':1,'A':1};
// Eşikler: [turuncu, kırmızı]. Yalnız ÖLÇÜLMÜŞ ya da şartnameden bilinen
// sınırlar var; tahmini eşik konmadı — renk uydurmak sayıyı çöpe çevirir.
const ESIK={yatis:[8,15],yunuslama:[8,15],hiz:[2.0,3.0],sicaklik:[70,80],
            cpu:[85,95],gpu:[85,95],yas:[1.0,3.0]};
function ond(bir){const o=OND[bir];return o===undefined?2:o}
function biçim(v,bir){
  if(v===null||v===undefined) return '—';
  return v.toFixed(ond(bir)).replace('.',',');
}
function sinif(k,v){
  if(v===null||v===undefined) return '';
  const e=ESIK[k]; if(!e) return '';
  const a=Math.abs(v);
  return a>=e[1]?'k':a>=e[0]?'t':'';
}

// ── Katalog + geçmiş ───────────────────────────────────────────────────────
let KATALOG=[];                 // [{k,ad,gr,bir,kaynak}]
const BILGI={};                 // k -> katalog satırı
const GECMIS={};                // k -> son N örnek (null dahil)
const N_GEC=180;                // 36 s @ 5 Hz
const cizgiler={};              // k -> canvas

fetch('/sinyaller').then(r=>r.json()).then(d=>{KATALOG=d;kur()})
  .catch(()=>{document.getElementById('kayGovde').innerHTML=
    '<div class=altbil>Sinyal kataloğu alınamadı.</div>'});

function kur(){
  const gov=document.getElementById('kayGovde');
  const gruplar=[];
  for(const s of KATALOG){
    BILGI[s.k]=s; GECMIS[s.k]=[];
    if(!gruplar.includes(s.gr)) gruplar.push(s.gr);
  }
  let yok=0;
  for(const g of gruplar){
    const kap=document.createElement('div'); kap.className='kayGrup';
    const bas=document.createElement('div');
    bas.className='kayGrupBas mikro'; bas.textContent=g;
    const tab=document.createElement('table'); tab.className='kay';
    for(const s of KATALOG.filter(x=>x.gr===g)){
      const tr=document.createElement('tr');
      if(!s.kaynak){tr.className='kaynaksiz'; yok++}
      const ad=document.createElement('td'); ad.className='kAd'; ad.textContent=s.ad;
      const cz=document.createElement('td'); cz.className='kCiz';
      const cv=document.createElement('canvas'); cv.width=1200; cv.height=44;
      cz.appendChild(cv); cizgiler[s.k]=cv;
      const dg=document.createElement('td'); dg.className='kDeg';
      dg.innerHTML='—<em>'+s.bir+'</em>';
      const ky=document.createElement('td'); ky.className='kKay';
      ky.textContent=s.kaynak||'kaynak yok'; ky.title=s.kaynak||'kaynak yok';
      // Ad → değer → eğilim → kaynak: önce ne olduğu, sonra kaç, sonra nereye
      // gittiği. Kaynak en sağda çünkü en seyrek bakılan sütun.
      tr.append(ad,dg,cz,ky); tab.appendChild(tr);
    }
    kap.append(bas,tab); gov.appendChild(kap);
  }
  document.getElementById('kaySayim').textContent =
    KATALOG.length+' sinyal · '+yok+' tanesinin bu araçta kaynağı yok';
}

// Kıvılcım çizgisi: null'lar boşluk bırakır, çizgi oradan kopar — donmuş
// sensör düz çizgi değil DELİK gösterir.
function ciz(k){
  const cv=cizgiler[k]; if(!cv) return;
  const g=GECMIS[k]||[], c=cv.getContext('2d');
  const W=cv.width,H=cv.height;
  c.clearRect(0,0,W,H);
  const s=g.filter(v=>v!==null);
  if(s.length<2) return;
  let mn=Math.min(...s), mx=Math.max(...s);
  if(mx-mn<1e-9){mn-=.5;mx+=.5}
  const st=document.documentElement;
  const renk=getComputedStyle(st).getPropertyValue('--y2').trim()||'#888';
  c.strokeStyle=renk; c.lineWidth=3; c.lineJoin='round'; c.lineCap='round';
  c.beginPath();
  let kalem=false;
  for(let i=0;i<g.length;i++){
    const v=g[i];
    if(v===null){kalem=false;continue}
    const x=(i/(N_GEC-1))*(W-4)+2;
    const y=H-4-((v-mn)/(mx-mn))*(H-8);
    if(kalem) c.lineTo(x,y); else {c.moveTo(x,y);kalem=true}
  }
  c.stroke();
}

// ── Telemetri akışı ────────────────────────────────────────────────────────
const es=new EventSource('/telemetri');
es.onmessage=e=>{
  let d; try{d=JSON.parse(e.data)}catch(_){return}
  const S=d.sinyal||{};

  // durum kapsülleri
  rz('rzBag', d.bagli?'bağlı':'bağlantı kesildi', d.bagli?'g':'k');
  rz('rzEstop', d.estop?'E-STOP AKTİF':'e-stop normal', d.estop?'k':'g');
  rz('rzMod', d.mod_ad||'—',
     d.mod===2?'g':d.mod===1?'t':d.mod===0?'t':'');

  // otonom görünümü metinleri
  metin('oFsm', d.fsm||'—');
  metin('oTgt', d.targeting||'—');
  metin('oLazer', d.shoot?'ATEŞ':'beklemede');

  // büyük sayılar
  for(const el of document.querySelectorAll('[data-dev]')){
    const k=el.dataset.dev, b=BILGI[k], v=S[k];
    const bir=b?b.bir:'';
    const em=el.querySelector('em');
    el.firstChild.nodeValue=biçim(v,bir);
    el.className='dev sayi '+(v===null||v===undefined?'yok':sinif(k,v));
    if(em) el.appendChild(em);
  }
  // satır değerleri
  for(const el of document.querySelectorAll('[data-sat]')){
    const k=el.dataset.sat, b=BILGI[k], v=S[k];
    const bir=b?b.bir:'';
    el.textContent = v===null||v===undefined ? '—' : biçim(v,bir)+' '+bir;
    el.className = v===null||v===undefined ? 'yok' : sinif(k,v);
  }
  // satır içi küçük değerler
  for(const el of document.querySelectorAll('[data-inline]')){
    const k=el.dataset.inline, b=BILGI[k];
    el.textContent=biçim(S[k], b?b.bir:'');
  }

  // kayıt tablosu + geçmiş
  const satirlar=document.querySelectorAll('#kayGovde tr');
  let i=0;
  for(const s of KATALOG){
    const v=(S[s.k]===undefined)?null:S[s.k];
    const g=GECMIS[s.k]; if(g){g.push(v); if(g.length>N_GEC) g.shift()}
    const tr=satirlar[i++]; if(!tr) continue;
    const dg=tr.querySelector('.kDeg');
    dg.innerHTML=biçim(v,s.bir)+'<em>'+s.bir+'</em>';
    dg.className='kDeg '+sinif(s.k,v);
  }
  if(document.getElementById('gKayit').classList.contains('aktif'))
    for(const s of KATALOG) ciz(s.k);
};

function rz(id,yazi,sf){
  const el=document.getElementById(id);
  el.textContent=yazi; el.className='rz '+(sf||'');
}
function metin(id,yazi){document.getElementById(id).textContent=yazi}

// ── Kameralar ──────────────────────────────────────────────────────────────
// Her kare ayrı istek. MJPEG bağlantıyı süresiz açık tutar; 6 panel + SSE
// tarayıcının 6 eşzamanlı bağlantı sınırını doldurur ve sayfa kilitlenir.
for(const alan of document.querySelectorAll('[data-kam]')){
  const ad=alan.dataset.kam, im=alan.querySelector('img');
  const fpsEl=document.querySelector('[data-kamfps="'+ad+'"]');
  let bekliyor=false, sayac=0;
  const cek=()=>{
    // Görünmeyen sekmedeki paneli çekmek Jetson'da boşuna JPEG kodlatır.
    if(bekliyor || !alan.closest('.gorunum').classList.contains('aktif')) return;
    bekliyor=true;
    const y=new Image();
    y.onload=()=>{im.src=y.src; alan.classList.add('canli'); bekliyor=false; sayac++};
    y.onerror=()=>{alan.classList.remove('canli'); bekliyor=false};
    y.src='/kare/'+ad+'?t='+Date.now();
  };
  cek(); setInterval(cek,100);
  if(fpsEl) setInterval(()=>{fpsEl.textContent=sayac+' fps'; sayac=0},1000);
}

// ── Görünüm anahtarı ───────────────────────────────────────────────────────
for(const d of document.querySelectorAll('.anahtar button')){
  d.onclick=()=>{
    for(const o of document.querySelectorAll('.anahtar button'))
      o.setAttribute('aria-pressed', String(o===d));
    for(const g of document.querySelectorAll('.gorunum'))
      g.classList.toggle('aktif', g.id===d.dataset.gor);
  };
}
addEventListener('keydown',e=>{
  if(e.target.tagName==='INPUT') return;
  const i={'1':0,'2':1,'3':2}[e.key];
  if(i!==undefined) document.querySelectorAll('.anahtar button')[i].click();
});

// ── E-STOP ─────────────────────────────────────────────────────────────────
document.getElementById('estopBas').onclick=()=>{
  fetch('/estop',{method:'POST',body:JSON.stringify({aktif:true})});
};
document.getElementById('estopKaldir').onclick=()=>{
  if(confirm('E-STOP GCS kaynağı kaldırılsın mı?\\nDiğer kaynaklar hâlâ aktifse durdurma sürer.'))
    fetch('/estop',{method:'POST',body:JSON.stringify({aktif:false})});
};

// ── Saat + kronometre ──────────────────────────────────────────────────────
setInterval(()=>{document.getElementById('saat').textContent=
  new Date().toLocaleTimeString('tr-TR')},1000);
let sT=0,sBas=0,sInt=null;
document.getElementById('sureD').onclick=()=>{
  if(sInt){clearInterval(sInt);sInt=null;sT+=Date.now()-sBas}
  else{sBas=Date.now();sInt=setInterval(sureCiz,100)}
};
document.getElementById('sureD').ondblclick=()=>{
  clearInterval(sInt);sInt=null;sT=0;sureCiz()
};
function sureCiz(){
  const ms=sT+(sInt?Date.now()-sBas:0);
  const s=Math.floor(ms/1000), m=Math.floor(s/60);
  document.getElementById('sure').textContent=
    String(m).padStart(2,'0')+':'+String(s%60).padStart(2,'0')+
    '.'+Math.floor(ms%1000/100);
}
</script></body></html>"""


# ── HTTP işleyici ─────────────────────────────────────────────────────────────
class Isleyici(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_GET(self):
        if self.path == '/' or self.path == '/index.html':
            self._sayfa(SAYFA_YENI)
        elif self.path == '/eski':
            # Sahada bir şey ters giderse geri dönülecek adres. Yeni pano
            # /sinyaller + /telemetri, eskisi /veri kullanıyor — ikisi de
            # sunucuda duruyor, biri diğerini bozmuyor.
            self._sayfa(SAYFA)
        elif self.path.startswith('/kare/'):
            self._tek_kare(self.path[len('/kare/'):].split('?')[0])
        elif self.path.startswith('/stream/'):
            self._mjpeg(self.path[len('/stream/'):])
        elif self.path == '/veri':
            self._sse()
        elif self.path == '/sinyaller':
            self._json(sinyal_katalog_json())
        elif self.path == '/telemetri':
            self._sse(telemetri_json)
        else:
            self.send_error(404)

    def do_POST(self):
        if self.path == '/estop':
            try:
                n = int(self.headers.get('Content-Length', 0))
                d = json.loads(self.rfile.read(n) or b'{}')
                DUGUM.gcs_estop(bool(d.get('aktif', False)))
                self.send_response(200)
                self.end_headers()
            except Exception:
                self.send_error(400)
        else:
            self.send_error(404)

    def _sayfa(self, metin):
        govde = metin.encode('utf-8')
        self.send_response(200)
        self.send_header('Content-Type', 'text/html; charset=utf-8')
        self.send_header('Content-Length', str(len(govde)))
        self.end_headers()
        try:
            self.wfile.write(govde)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _tek_kare(self, ad):
        """Tek JPEG döner ve bağlantıyı kapatır.

        MJPEG akışı bağlantıyı süresiz açık tutar; tarayıcılar aynı sunucuya
        en fazla 6 eşzamanlı bağlantı açtığı için 6 panel + SSE havuzu
        doldurur ve sayfa kilitlenir. Kare bazlı çekimde her istek
        milisaniyeler içinde kapanır, havuz serbest kalır.
        """
        if ad not in PANELLER:
            self.send_error(404)
            return
        bgr = panel_kare(ad)
        jpg = kucult_kodla(bgr) if bgr is not None else None
        if jpg is None:
            self.send_error(503)
            return
        self.send_response(200)
        self.send_header('Content-Type', 'image/jpeg')
        self.send_header('Content-Length', str(len(jpg)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        try:
            self.wfile.write(jpg)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _mjpeg(self, ad):
        gecerli = PANELLER
        if ad not in gecerli:
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header('Age', '0')
        self.send_header('Cache-Control', 'no-cache, private')
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=kare')
        self.end_headers()
        try:
            self.wfile.write(b'\r\n')
            son = 0.0
            while True:
                bgr = panel_kare(ad)
                if bgr is None:
                    time.sleep(0.2)
                    continue
                # fps sınırı
                dt = time.time() - son
                if dt < 1.0 / AKIS_FPS:
                    time.sleep(1.0 / AKIS_FPS - dt)
                son = time.time()
                jpg = kucult_kodla(bgr)
                if jpg is None:
                    continue
                self.wfile.write(b'--kare\r\n')
                self.wfile.write(b'Content-Type: image/jpeg\r\n')
                self.wfile.write(('Content-Length: %d\r\n\r\n' % len(jpg)).encode())
                self.wfile.write(jpg)
                self.wfile.write(b'\r\n')
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _json(self, govde):
        ham = json.dumps(govde, allow_nan=False).encode('utf-8')
        self.send_response(200)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(ham)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        try:
            self.wfile.write(ham)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _sse(self, uretici=None):
        uretici = uretici or sensor_json
        self.send_response(200)
        self.send_header('Content-Type', 'text/event-stream')
        self.send_header('Cache-Control', 'no-cache')
        self.send_header('Connection', 'keep-alive')
        self.end_headers()
        try:
            while True:
                # allow_nan=False: NaN geçerli JSON değil, JSON.parse patlar ve
                # akış sessizce ölür. Bir sensör NaN üretirse o kare atlanır.
                try:
                    veri = json.dumps(uretici(), allow_nan=False)
                except ValueError:
                    time.sleep(0.2)
                    continue
                self.wfile.write(('data: %s\n\n' % veri).encode('utf-8'))
                self.wfile.flush()
                time.sleep(0.2)   # 5 Hz
        except (BrokenPipeError, ConnectionResetError):
            pass


DUGUM = None


def main():
    global DUGUM
    rclpy.init()
    DUGUM = WebDashboardNode()
    threading.Thread(target=rclpy.spin, args=(DUGUM,), daemon=True).start()
    srv = ThreadingHTTPServer(('0.0.0.0', PORT), Isleyici)
    DUGUM.get_logger().info(f'Web dashboard hazır: http://<jetson-ip>:{PORT}')
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        srv.shutdown()
        DUGUM.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
