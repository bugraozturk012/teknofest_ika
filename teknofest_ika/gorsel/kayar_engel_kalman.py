#!/usr/bin/env python3
"""
kayar_engel_kalman.py — Kayar Engel Kalman Filtresi
====================================================
Şartname 6.8: Kayar engel 20 cm/s sabit hızla hareket eder.

Çalışma prensibi:
  1. /scan (LaserScan) üzerinden engel lateral konumu ölçülür
  2. 1D Kalman filtresi ile konum ve hız tahmini yapılır
  3. /moving_obs/prediction yayınlanır (Point: x=konum_m, y=hiz_ms, z=0)

Durum vektörü: [y_konum, y_hiz]  (araç koordinat sistemi, y=lateral)
Süreç modeli : sabit hız (constant velocity)
Ölçüm        : LiDAR'dan engelin lateral konumu

Test:
  ros2 topic echo /moving_obs/prediction
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point

SCAN_TOPIC   = '/scan'
OUTPUT_TOPIC = '/moving_obs/prediction'

# Engel arama penceresi (araç önünde, ±60°, 0.5–4 m arası)
ENGEL_MIN_MESAFE = 0.5
ENGEL_MAX_MESAFE = 4.0
ENGEL_ACI_MIN    = -math.radians(60)
ENGEL_ACI_MAX    =  math.radians(60)


class KayarEngelKalman(Node):

    def __init__(self):
        super().__init__('kayar_engel_kalman')

        # Kalman durum vektörü: [y_konum [m], y_hiz [m/s]]
        self._x = np.zeros(2)
        self._P = np.eye(2) * 1.0

        # Süreç gürültüsü (Q) — sabit hız modeli
        self._Q = np.diag([0.01, 0.1])
        # Ölçüm gürültüsü (R) — YDLidar Tmini Pro ~3 cm std
        self._R = np.array([[0.03]])
        # Ölçüm matrisi (sadece konum gözlemlenir)
        self._H = np.array([[1.0, 0.0]])

        self._initialized = False
        self._last_time   = 0.0

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(LaserScan, SCAN_TOPIC, self._scan_cb, qos)
        self._pub = self.create_publisher(Point, OUTPUT_TOPIC, 10)

        self.get_logger().info(
            f'KayarEngelKalman hazır | '
            f'mesafe=[{ENGEL_MIN_MESAFE},{ENGEL_MAX_MESAFE}] m | '
            f'açı=[{math.degrees(ENGEL_ACI_MIN):.0f}°,{math.degrees(ENGEL_ACI_MAX):.0f}°]'
        )

    def _scan_cb(self, msg: LaserScan):
        now = self.get_clock().now().nanoseconds / 1e9

        olcum = self._engel_bul(msg)
        if olcum is None:
            return

        if not self._initialized:
            self._x[:] = [olcum, 0.0]
            self._P    = np.eye(2) * 1.0
            self._initialized = True
            self._last_time   = now
            return

        dt = now - self._last_time
        if dt <= 0.0:
            return
        self._last_time = now

        # ── Tahmin (Predict) ──────────────────────────────────────────────
        F = np.array([[1.0, dt],
                      [0.0, 1.0]])
        self._x = F @ self._x
        self._P = F @ self._P @ F.T + self._Q

        # ── Güncelleme (Update) ───────────────────────────────────────────
        S   = self._H @ self._P @ self._H.T + self._R
        K   = self._P @ self._H.T / S[0, 0]
        inn = olcum - self._H @ self._x

        self._x = self._x + K.flatten() * inn[0]
        self._P = (np.eye(2) - K.reshape(2, 1) @ self._H) @ self._P

        # ── Yayın ─────────────────────────────────────────────────────────
        out = Point()
        out.x = float(self._x[0])   # lateral konum [m]
        out.y = float(self._x[1])   # lateral hız   [m/s]
        out.z = 0.0
        self._pub.publish(out)

        self.get_logger().debug(
            f'konum={self._x[0]:.3f} m  hız={self._x[1]:.3f} m/s',
            throttle_duration_sec=0.5,
        )

    def _engel_bul(self, msg: LaserScan):
        """Arama penceresindeki en yakın noktanın lateral konumunu döndürür."""
        en_yakin_r = float('inf')
        en_yakin_y = None

        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r):
                continue
            if r < ENGEL_MIN_MESAFE or r > ENGEL_MAX_MESAFE:
                continue
            aci = msg.angle_min + i * msg.angle_increment
            if aci < ENGEL_ACI_MIN or aci > ENGEL_ACI_MAX:
                continue
            if r < en_yakin_r:
                en_yakin_r = r
                en_yakin_y = r * math.sin(aci)

        return en_yakin_y


def main(args=None):
    rclpy.init(args=args)
    node = KayarEngelKalman()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
