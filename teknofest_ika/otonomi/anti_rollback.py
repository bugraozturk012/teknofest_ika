#!/usr/bin/env python3
"""
anti_rollback.py — Geri Kayma Önleme Kontrolcüsü

Dik eğimde aracın geri kaymasını tespit eder ve önler.
Tespit: IMU pitch > 10° VE odom hız < -0.05 m/s

Komut mimarisi (race condition yoktur):
  /anti_rollback/cmd   (Twist) → ackermann_converter okur, aktifken override yapar
  /anti_rollback/aktif (Bool)  → ackermann_converter bu flag'e göre karar verir
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

RAMP_PITCH_THRESHOLD   = math.radians(10.0)
ROLLBACK_VEL_THRESHOLD = 0.05
RECOVERY_SPEED         = 0.3
KONTROL_HZ             = 20.0


class AntiRollback(Node):

    def __init__(self):
        super().__init__('anti_rollback')

        self._pitch    = 0.0
        self._velocity = 0.0
        self._aktif    = False

        qos_be  = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        qos_rel = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self.create_subscription(Imu,      '/imu/data', self._imu_cb,  qos_be)
        self.create_subscription(Odometry, '/odom',     self._odom_cb, qos_be)

        # Twist komutu → ackermann_converter override eder (tek yazıcı garantisi)
        self._cmd_pub   = self.create_publisher(Twist, '/anti_rollback/cmd', qos_rel)
        self._durum_pub = self.create_publisher(Bool,  '/anti_rollback/aktif', 10)

        self.create_timer(1.0 / KONTROL_HZ, self._kontrol)
        self.get_logger().info(
            f'AntiRollback hazır | '
            f'pitch_esik={math.degrees(RAMP_PITCH_THRESHOLD):.0f}° | '
            f'vel_esik={ROLLBACK_VEL_THRESHOLD} m/s | '
            f'recovery={RECOVERY_SPEED} m/s'
        )

    def _imu_cb(self, msg: Imu):
        q    = msg.orientation
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        sinp = max(-1.0, min(1.0, sinp))
        self._pitch = math.asin(sinp)

    def _odom_cb(self, msg: Odometry):
        self._velocity = msg.twist.twist.linear.x

    def _kontrol(self):
        rollback = (self._pitch > RAMP_PITCH_THRESHOLD and
                    self._velocity < -ROLLBACK_VEL_THRESHOLD)

        if rollback and not self._aktif:
            self.get_logger().warn(
                f'[AntiRollback] GERİ KAYMA! '
                f'pitch={math.degrees(self._pitch):.1f}° '
                f'vel={self._velocity:.3f} m/s → {RECOVERY_SPEED} m/s override'
            )
        elif not rollback and self._aktif:
            self.get_logger().info('[AntiRollback] Geri kayma sona erdi.')

        self._aktif = rollback
        self._durum_pub.publish(Bool(data=self._aktif))

        if self._aktif:
            cmd = Twist()
            cmd.linear.x  = RECOVERY_SPEED
            cmd.angular.z = 0.0
            self._cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = AntiRollback()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
