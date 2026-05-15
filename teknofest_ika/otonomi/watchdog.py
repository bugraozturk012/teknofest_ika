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
  /scan               -> LiDAR      (timeout: 2.0s)
  /odom                -> Enkoder    (timeout: 1.0s)
  /imu/data            -> IMU        (timeout: 1.0s)
  /battery/status      -> Batarya    (timeout: 5.0s)
  /odometry/filtered   -> EKF        (timeout: 2.0s)

NOT: AnyMsg kullanir — mesaj tipinden bagimsiz, sadece topic
     canliligini kontrol eder. Deserialization yapmaz.

TEST:
  ros2 topic echo /sensor/fault
"""

import threading

import rclpy
from rclpy.node import Node
from rclpy.msg import AnyMsg
from std_msgs.msg import String

from teknofest_ika.otonomi.topics import (
    SCAN_TOPIC, SCAN_FILTERED_TOPIC, ODOM_TOPIC, IMU_TOPIC,
    BATTERY_TOPIC, EKF_ODOM_TOPIC, CAMERA_PROCESSED_TOPIC,
    YOLO_RAW_TOPIC, DETECTIONS_TOPIC, CONE_FUSION_CLOUD_TOPIC,
)

CRITICAL_TOPICS = {
    SCAN_TOPIC:              ('LiDAR',          2.0),
    SCAN_FILTERED_TOPIC:     ('LiDARFiltre',    3.0),
    ODOM_TOPIC:              ('Enkoder',        1.0),
    IMU_TOPIC:               ('IMU',            1.0),
    BATTERY_TOPIC:           ('Batarya',        5.0),
    EKF_ODOM_TOPIC:          ('EKF',            2.0),
    CAMERA_PROCESSED_TOPIC:  ('KameraOnIsleme', 2.0),
    YOLO_RAW_TOPIC:          ('YOLO',           3.0),
    DETECTIONS_TOPIC:        ('YOLOAdapter',    3.0),
    CONE_FUSION_CLOUD_TOPIC: ('ConeFusion',     5.0),
    # /targeting/error KASITLI OLARAK ÇIKARILDI:
    # targeting_node sadece misyon_fsm SHOOT_APPROACH state'inde aktif.
    # Navigasyon boyunca (~14 dk) bu topic sessiz kalır → sürekli yanlış alarm üretir.
}

STARTUP_GRACE_S = 15.0   # Bu surede tum topic'ler en az 1 kez gelmeli


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
                AnyMsg, topic,
                lambda msg, t=topic: self._cb(t),
                10)

        self.create_timer(1.0, self._check)

        self.get_logger().info(
            f'Watchdog hazir | {len(CRITICAL_TOPICS)} topic izleniyor | '
            f'AnyMsg (tip bagimsiz)'
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
