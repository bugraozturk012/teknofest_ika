#!/usr/bin/env python3
"""
kayar_engel_costmap.py — Kayar Engel → Nav2 Costmap Köprüsü
=============================================================
Kayar engel Kalman filtresi tarafından tespit edildiğinde (direction != 'bilinmiyor')
/scan noktalarını Nav2 ObstacleLayer'a PointCloud2 olarak bildirir.
Tespit durunca yayın durur — Nav2 clearing:true ile eski noktaları temizler.

koni_costmap.py'den farklar:
  - /scan kaynaklı, görüntü ekibine bağımsız
  - clearing: true (engel geçince Nav2 siler)
  - Yalnızca Kalman aktifken (/moving_obs/direction != 'bilinmiyor') yayın yapar

GİRİŞ : /scan                 (sensor_msgs/LaserScan)
         /moving_obs/direction (std_msgs/String) — Kalman aktif mi?
ÇIKIŞ : /moving_obs_cloud     (sensor_msgs/PointCloud2) — Nav2 ObstacleLayer
"""
import math
import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import String

# Kayar engel arama penceresi — kayar_engel_kalman.py ile aynı değerler
ENGEL_MIN_MESAFE = 0.5
ENGEL_MAX_MESAFE = 4.0
ENGEL_ACI_MIN    = -math.radians(60)
ENGEL_ACI_MAX    =  math.radians(60)

# Tek gürültü noktasını eleman saymamak için minimum nokta sayısı
MIN_NOKTA = 3


def _xyz_cloud(points_xy, frame_id: str, stamp) -> PointCloud2:
    """[(x,y), ...] → XYZ PointCloud2 (z=0, zemin seviyesi)."""
    fields = [
        PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
    ]
    data = bytearray()
    for x, y in points_xy:
        data += struct.pack('fff', x, y, 0.0)

    cloud = PointCloud2()
    cloud.header.stamp    = stamp
    cloud.header.frame_id = frame_id
    cloud.height          = 1
    cloud.width           = len(points_xy)
    cloud.fields          = fields
    cloud.is_bigendian    = False
    cloud.point_step      = 12
    cloud.row_step        = 12 * len(points_xy)
    cloud.data            = bytes(data)
    cloud.is_dense        = True
    return cloud


class KayarEngelCostmap(Node):

    def __init__(self):
        super().__init__('kayar_engel_costmap')

        # Kalman filtresi aktif mi? (direction != 'bilinmiyor')
        self._engel_aktif = False

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.create_subscription(LaserScan, '/scan', self._scan_cb, qos)
        self.create_subscription(
            String, '/moving_obs/direction', self._dir_cb, 10)

        self._pub = self.create_publisher(PointCloud2, '/moving_obs_cloud', 10)

        self.get_logger().info(
            'KayarEngelCostmap hazır | '
            '/scan + /moving_obs/direction → /moving_obs_cloud'
        )

    def _dir_cb(self, msg: String):
        self._engel_aktif = (msg.data != 'bilinmiyor')

    def _scan_cb(self, msg: LaserScan):
        if not self._engel_aktif:
            return

        noktalar = []
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r):
                continue
            if r < ENGEL_MIN_MESAFE or r > ENGEL_MAX_MESAFE:
                continue
            aci = msg.angle_min + i * msg.angle_increment
            if aci < ENGEL_ACI_MIN or aci > ENGEL_ACI_MAX:
                continue
            x = r * math.cos(aci)
            y = r * math.sin(aci)
            noktalar.append((x, y))

        if len(noktalar) < MIN_NOKTA:
            return

        frame = msg.header.frame_id if msg.header.frame_id else 'laser'
        cloud = _xyz_cloud(noktalar, frame, msg.header.stamp)
        self._pub.publish(cloud)

        self.get_logger().debug(
            f'Kayar engel: {len(noktalar)} nokta costmap\'e eklendi',
            throttle_duration_sec=1.0
        )


def main(args=None):
    rclpy.init(args=args)
    node = KayarEngelCostmap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
