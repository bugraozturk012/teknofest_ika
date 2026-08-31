#!/usr/bin/env python3
"""
watchdog.py — LYDIA IKA Sensor Saglik ve Baslangic Kontrolu
=============================================================
Launch sonrasi kritik topic'lerin canli olup olmadigini denetler.

GOREVLERI:
  1. Baslangic saglik kontrolu: Tum topic'ler belirli surede
     en az bir kez yayinlanmazsa uyari verir.
  2. Surekli heartbeat: Her topic'in son yayin zamanini takip eder,
     timeout'ta /sensor/fault yayinlar.

DENETLENEN TOPIC'LER (liste calisma aninda kuruluyor):
  her zaman:
    <ham_tarama_topic>   -> LiDAR      (timeout: 2.0s, varsayilan /scan)
    /scan/filtered       -> LiDARFiltre(timeout: 3.0s)
    /odom                -> Enkoder    (timeout: 1.0s)
    /imu/data            -> IMU        (timeout: 1.0s)
    /battery/status      -> Batarya    (timeout: 5.0s)
    /camera/image_processed -> Kamera  (timeout: 2.0s)
    /detections/yolo     -> YOLO       (timeout: 3.0s)
    /e_stop              -> EStopNode  (timeout: 0.5s)
  yalniz nav2_aktif:=true iken:
    /odometry/filtered   -> EKF        (timeout: 2.0s)
    /ika/detections      -> YOLOAdapter(timeout: 3.0s)

PARAMETRELER:
  nav2_aktif       : Nav2 yolu acik mi (EKF + yolo_adapter ayakta mi)
  ham_tarama_topic : surucunun ham taramayi bastigi ad. Boot betiginde
                     /scan, launch dosyasinda /scan_lidar (scan_relay).
                     Yanlis ad KALICI sahte arizaya yol acar.

NOT: raw=True abonelik kullanir — mesaj icerigini deserialize etmez,
     sadece topic canliligini kontrol eder.

TEST:
  ros2 topic echo /sensor/fault
"""

import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import LaserScan, Imu, BatteryState, Image
from nav_msgs.msg import Odometry
from vision_msgs.msg import Detection2DArray

from teknofest_ika.otonomi.topics import (
    SCAN_TOPIC, SCAN_FILTERED_TOPIC, ODOM_TOPIC, IMU_TOPIC,
    BATTERY_TOPIC, EKF_ODOM_TOPIC, CAMERA_PROCESSED_TOPIC,
    YOLO_RAW_TOPIC, DETECTIONS_TOPIC, E_STOP_TOPIC, SENSOR_FAULT_TOPIC,
)

# Ham taramanın topic ADI başlatma yoluna göre değişiyor, bu yüzden burada
# sabit değil parametre:
#   boot betiği : ydlidar sürücüsü doğrudan /scan basar (preprocessing
#                 `-r /scan_lidar:=/scan` ile ona abone olur)
#   launch      : sürücü /scan_raw'a remap edilir, scan_relay /scan_lidar basar
# Sabit /scan_lidar yazılıydı; sahada koşan yol boot betiği olduğu için
# watchdog her açılışta "LiDAR henüz veri gelmedi" diye KALICI sahte alarm
# veriyordu. Sahte alarm, kontrolün kendisini değersizleştirdiği için gerçek
# arızayı kaçırmakla aynı sonucu verir.
VARSAYILAN_HAM_TARAMA = SCAN_TOPIC

# Her zaman izlenenler — bunların yayıncısı başlatma yolundan bağımsız çalışır.
TEMEL_TOPICLER = {
    SCAN_FILTERED_TOPIC:     ('LiDARFiltre',    3.0),
    ODOM_TOPIC:              ('Enkoder',        1.0),
    IMU_TOPIC:               ('IMU',            1.0),
    BATTERY_TOPIC:           ('Batarya',        5.0),
    CAMERA_PROCESSED_TOPIC:  ('KameraOnIsleme', 2.0),
    YOLO_RAW_TOPIC:          ('YOLO',           3.0),
    # Güvenlik alt sisteminin kendi canlılığı da izlenir (Şartname §6.13/§7.8
    # ayrılık ilkesi): e_stop_node 20Hz yayın yapar (bkz. e_stop_node.py),
    # bu yayın kesilirse mod_yoneticisi/seri_kopru "son bilinen False" ile
    # sessizce çalışmaya devam edebilir — watchdog bunu /sensor/fault ile açığa çıkarır.
    E_STOP_TOPIC:            ('EStopNode',      0.5),
    # CONE_FUSION_CLOUD_TOPIC KASITLI OLARAK ÇIKARILDI:
    # cone_fusion_node yalnızca koni tespit edilince yayın yapar.
    # Parkurun büyük bölümünde koni yok → sürekli yanlış alarm üretir.
    # /targeting/error DA ÇIKARILDI:
    # targeting_node sadece misyon_fsm SHOOT_APPROACH state'inde aktif.
}

# Yalnız NAV2_AKTIF=1 iken yayıncısı ayağa kalkanlar. Nav2 kapalıyken
# izlenirlerse kalıcı sahte arıza üretirler: EKF de yolo_adapter da boot
# betiğinde Nav2 bloğunun içinde başlatılıyor.
NAV2_TOPICLERI = {
    EKF_ODOM_TOPIC:          ('EKF',            2.0),
    DETECTIONS_TOPIC:        ('YOLOAdapter',    3.0),
}

# ROS2 Humble'da rclpy.msg.AnyMsg diye bir şey yok (ROS1'e özgüydü) — her
# topic'in gerçek mesaj tipiyle raw=True abonelik aynı sonucu veriyor
# (içerik deserialize edilmiyor, sadece canlılık takip ediliyor).
_TOPIC_MSG_TYPE = {
    SCAN_FILTERED_TOPIC:    LaserScan,
    ODOM_TOPIC:             Odometry,
    IMU_TOPIC:              Imu,
    BATTERY_TOPIC:          BatteryState,
    EKF_ODOM_TOPIC:         Odometry,
    CAMERA_PROCESSED_TOPIC: Image,
    YOLO_RAW_TOPIC:         Detection2DArray,
    DETECTIONS_TOPIC:       String,
    E_STOP_TOPIC:           Bool,
}

STARTUP_GRACE_S = 30.0   # YOLO TensorRT engine yükleme süresi (~5-10s) dahil


class Watchdog(Node):

    def __init__(self):
        super().__init__('watchdog')

        # İzlenecek liste çalışma anında kuruluyor: hangi düğümlerin ayakta
        # olduğu başlatma yoluna ve NAV2_AKTIF'e bağlı. Sabit liste, kapalı
        # bir alt sistemi "arızalı" diye raporluyordu.
        self.declare_parameter('nav2_aktif', False)
        self.declare_parameter('ham_tarama_topic', VARSAYILAN_HAM_TARAMA)
        nav2_aktif  = bool(self.get_parameter('nav2_aktif').value)
        ham_tarama  = str(self.get_parameter('ham_tarama_topic').value)

        self._topicler = dict(TEMEL_TOPICLER)
        if ham_tarama:
            self._topicler[ham_tarama] = ('LiDAR', 2.0)
            _TOPIC_MSG_TYPE[ham_tarama] = LaserScan
        if nav2_aktif:
            self._topicler.update(NAV2_TOPICLERI)

        self._lock = threading.Lock()
        self._last_seen = {}
        self._all_ok = False
        self._start_time = self.get_clock().now().nanoseconds / 1e9

        self._fault_pub = self.create_publisher(String, SENSOR_FAULT_TOPIC, 10)

        for topic in self._topicler:
            self._last_seen[topic] = 0.0
            self.create_subscription(
                _TOPIC_MSG_TYPE[topic], topic,
                lambda msg, t=topic: self._cb(t),
                10, raw=True)

        self.create_timer(1.0, self._check)

        self.get_logger().info(
            f'Watchdog hazir | {len(self._topicler)} topic izleniyor '
            f'(nav2_aktif={nav2_aktif}, ham_tarama={ham_tarama}) | '
            f'raw=True (deserialize edilmiyor)'
        )

    def _cb(self, topic: str):
        with self._lock:
            self._last_seen[topic] = self.get_clock().now().nanoseconds / 1e9

    def _check(self):
        now = self.get_clock().now().nanoseconds / 1e9
        faults = []

        with self._lock:
            seen = {t: self._last_seen[t] for t in self._topicler}

        for topic, (name, timeout) in self._topicler.items():
            elapsed = now - seen[topic]
            if seen[topic] == 0.0:
                if now - self._start_time < STARTUP_GRACE_S:
                    continue
                msg = f'{name}({topic}) henuz veri gelmedi'
                faults.append(msg)
            elif elapsed > timeout:
                msg = f'{name}({topic}) {elapsed:.1f}s sessiz'
                faults.append(msg)
                self.get_logger().error(msg, throttle_duration_sec=5.0)

        self._fault_pub.publish(
            String(data='; '.join(faults) if faults else 'OK'))

        if not faults and not self._all_ok:
            self._all_ok = True
            self.get_logger().info(
                f'Tum {len(self._topicler)} kritik topic canli. '
                'Sistem hazir.')


def main(args=None):
    rclpy.init(args=args)
    node = Watchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
