#!/usr/bin/env python3
"""
imu_guvenlik.py — IMU Tabanlı Hız Güvenlik Kontrolcüsü
=======================================================
IMU roll/pitch değerlerini izleyerek /speed_limit topic'ine
hız sınırı yayınlar. terrain_adapter bu sınırı Nav2'ye uygular.

Kurallar:
  |roll| > 8°  → hızı doğrusal düşür (8°=yarı hız, 15°=dur)
  |roll| > 15° → DUR (0 m/s)
  pitch < -15° → yokuş aşağı fren modu (0.4 m/s)
  diğer        → normal hız (NORMAL_MAX_HIZ)

Eşikler topics.py ile eşleşmeli:
  IMU_ROLL_WARN_THRESHOLD  = 8.0°
  IMU_ROLL_STOP_THRESHOLD  = 15.0°
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import Imu
from std_msgs.msg import Float32

IMU_ROLL_WARN_THRESHOLD  =  8.0    # derece
IMU_ROLL_STOP_THRESHOLD  = 15.0   # derece
IMU_PITCH_DOWN_THRESHOLD = 15.0   # derece (yokuş aşağı)

NORMAL_MAX_HIZ    = 2.0   # [m/s] — terrain_adapter normal profiliyle eşleşmeli
FRENLEME_HIZ      = 0.4   # [m/s] — yokuş aşağı güvenli hız
YAYINLAMA_HZ      = 10.0


class ImuGuvenlik(Node):

    def __init__(self):
        super().__init__('imu_guvenlik')

        self._roll  = 0.0
        self._pitch = 0.0

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Imu, '/imu/data', self._imu_cb, qos)
        self._pub = self.create_publisher(Float32, '/speed_limit', 10)

        self.create_timer(1.0 / YAYINLAMA_HZ, self._yayinla)
        self.get_logger().info(
            f'ImuGuvenlik hazır | '
            f'roll_uyari={IMU_ROLL_WARN_THRESHOLD}° | '
            f'roll_dur={IMU_ROLL_STOP_THRESHOLD}°'
        )

    def _imu_cb(self, msg: Imu):
        q = msg.orientation

        # Quaternion → roll (ZYX Euler, X ekseni)
        sinr = 2.0 * (q.w * q.x + q.y * q.z)
        cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        self._roll = math.degrees(math.atan2(sinr, cosr))

        # Quaternion → pitch (Y ekseni)
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        sinp = max(-1.0, min(1.0, sinp))
        self._pitch = math.degrees(math.asin(sinp))

    def _yayinla(self):
        roll_abs = abs(self._roll)

        if roll_abs >= IMU_ROLL_STOP_THRESHOLD:
            hiz = 0.0
            self.get_logger().warn(
                f'[ImuGuvenlik] DUR — roll={self._roll:.1f}°',
                throttle_duration_sec=1.0,
            )
        elif roll_abs >= IMU_ROLL_WARN_THRESHOLD:
            # 8°→%50 hız, 15°→0 hız (doğrusal interpolasyon)
            oran = 1.0 - (roll_abs - IMU_ROLL_WARN_THRESHOLD) / (
                IMU_ROLL_STOP_THRESHOLD - IMU_ROLL_WARN_THRESHOLD)
            hiz = NORMAL_MAX_HIZ * 0.5 * max(0.0, oran)
            self.get_logger().warn(
                f'[ImuGuvenlik] Yan eğim — roll={self._roll:.1f}° → {hiz:.2f} m/s',
                throttle_duration_sec=2.0,
            )
        elif self._pitch <= -IMU_PITCH_DOWN_THRESHOLD:
            hiz = FRENLEME_HIZ
            self.get_logger().warn(
                f'[ImuGuvenlik] Yokuş aşağı — pitch={self._pitch:.1f}° → {hiz} m/s',
                throttle_duration_sec=2.0,
            )
        else:
            hiz = NORMAL_MAX_HIZ

        msg = Float32()
        msg.data = float(hiz)
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuGuvenlik()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
