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

DENETLENEN TOPIC'LER:
  /scan_lidar          -> LiDAR      (timeout: 2.0s)
  /scan/filtered       -> LiDARFiltre(timeout: 3.0s)
  /odom                -> Enkoder    (timeout: 1.0s)
  /imu/data            -> IMU        (timeout: 1.0s)
  /battery/status      -> Batarya    (timeout: 5.0s)
  /odometry/filtered   -> EKF        (timeout: 2.0s)
  /camera/image_processed -> Kamera  (timeout: 2.0s)
  /detections/yolo     -> YOLO       (timeout: 3.0s)
  /ika/detections      -> YOLOAdapter(timeout: 3.0s)

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
    SCAN_LIDAR_TOPIC, SCAN_FILTERED_TOPIC, ODOM_TOPIC, IMU_TOPIC,
    BATTERY_TOPIC, EKF_ODOM_TOPIC, CAMERA_PROCESSED_TOPIC,
    YOLO_RAW_TOPIC, DETECTIONS_TOPIC, E_STOP_TOPIC,
)

CRITICAL_TOPICS = {
    # /scan (/scan_raw → scan_relay → /scan_lidar) — SCAN_TOPIC ("/scan") KALDIRILDI:
    # Hiçbir node /scan'e doğrudan yayın yapmıyor; scan_relay /scan_lidar'a yayınlıyor.
    SCAN_LIDAR_TOPIC:        ('LiDAR',          2.0),
    SCAN_FILTERED_TOPIC:     ('LiDARFiltre',    3.0),
    ODOM_TOPIC:              ('Enkoder',        1.0),
    IMU_TOPIC:               ('IMU',            1.0),
    BATTERY_TOPIC:           ('Batarya',        5.0),
    EKF_ODOM_TOPIC:          ('EKF',            2.0),
    CAMERA_PROCESSED_TOPIC:  ('KameraOnIsleme', 2.0),
    YOLO_RAW_TOPIC:          ('YOLO',           3.0),
    DETECTIONS_TOPIC:        ('YOLOAdapter',    3.0),
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

# ROS2 Humble'da rclpy.msg.AnyMsg diye bir şey yok (ROS1'e özgüydü) — her
# topic'in gerçek mesaj tipiyle raw=True abonelik aynı sonucu veriyor
# (içerik deserialize edilmiyor, sadece canlılık takip ediliyor).
_TOPIC_MSG_TYPE = {
    SCAN_LIDAR_TOPIC:       LaserScan,
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
        self._lock = threading.Lock()
        self._last_seen = {}
        self._all_ok = False
        self._start_time = self.get_clock().now().nanoseconds / 1e9

        self._fault_pub = self.create_publisher(String, '/sensor/fault', 10)

        for topic in CRITICAL_TOPICS:
            self._last_seen[topic] = 0.0
            self.create_subscription(
                _TOPIC_MSG_TYPE[topic], topic,
                lambda msg, t=topic: self._cb(t),
                10, raw=True)

        self.create_timer(1.0, self._check)

        self.get_logger().info(
            f'Watchdog hazir | {len(CRITICAL_TOPICS)} topic izleniyor | '
            f'raw=True (deserialize edilmiyor)'
        )

    def _cb(self, topic: str):
        with self._lock:
            self._last_seen[topic] = self.get_clock().now().nanoseconds / 1e9

    def _check(self):
        now = self.get_clock().now().nanoseconds / 1e9
        faults = []

        with self._lock:
            seen = {t: self._last_seen[t] for t in CRITICAL_TOPICS}

        for topic, (name, timeout) in CRITICAL_TOPICS.items():
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
                f'Tum {len(CRITICAL_TOPICS)} kritik topic canli. '
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
