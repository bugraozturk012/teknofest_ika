#!/usr/bin/env python3
"""
kayar_engel_costmap.py — Kayar Engel → Nav2 Costmap Köprüsü
=============================================================
Kayar engel Kalman filtresi tarafından tespit edildiğinde (direction != 'bilinmiyor')
/scan noktalarını Nav2 ObstacleLayer'a PointCloud2 olarak bildirir.
Tespit durunca yayın durur — Nav2 clearing:true ile eski noktaları temizler.

Kaynağı /scan olduğu için görüntü zincirinden bağımsız çalışır; koni
köprüsünün (cone_fusion_node) aksine kameraya ihtiyaç duymaz.

GİRİŞ : /scan                 (sensor_msgs/LaserScan)
         /moving_obs/direction (std_msgs/String) — Kalman aktif mi?
ÇIKIŞ : /moving_obs_cloud     (sensor_msgs/PointCloud2) — Nav2 ObstacleLayer
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import String

from teknofest_ika.otonomi.topics import (
    SCAN_FILTERED_TOPIC, MOVING_OBS_DIR_TOPIC, MOVING_OBS_CLOUD_TOPIC,
)

# Kayar engel arama penceresi — kayar_engel_kalman.py ile aynı değerler
ENGEL_MIN_MESAFE = 0.5
ENGEL_MAX_MESAFE = 4.0
ENGEL_ACI_MIN    = -math.radians(60)
ENGEL_ACI_MAX    =  math.radians(60)

# Tek gürültü noktasını eleman saymamak için minimum nokta sayısı
MIN_NOKTA = 3

_FIELDS = [
    PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
]


def _xyz_cloud(xs: np.ndarray, ys: np.ndarray, frame_id: str, stamp) -> PointCloud2:
    n = len(xs)
    zs = np.zeros(n, dtype=np.float32)
    data = np.column_stack([xs.astype(np.float32),
                            ys.astype(np.float32),
                            zs]).tobytes()

    cloud = PointCloud2()
    cloud.header.stamp    = stamp
    cloud.header.frame_id = frame_id
    cloud.height          = 1
    cloud.width           = n
    cloud.fields          = _FIELDS
    cloud.is_bigendian    = False
    cloud.point_step      = 12
    cloud.row_step        = 12 * n
    cloud.data            = data
    cloud.is_dense        = True
    return cloud


class KayarEngelCostmap(Node):

    def __init__(self):
        super().__init__('kayar_engel_costmap')

        self._engel_aktif = False

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.create_subscription(LaserScan, SCAN_FILTERED_TOPIC, self._scan_cb, qos)
        self.create_subscription(
            String, MOVING_OBS_DIR_TOPIC, self._dir_cb, 10)

        self._pub = self.create_publisher(PointCloud2, MOVING_OBS_CLOUD_TOPIC, 10)

        self.get_logger().info(
            'KayarEngelCostmap hazır | '
            '/scan + /moving_obs/direction → /moving_obs_cloud'
        )

    def _dir_cb(self, msg: String):
        self._engel_aktif = (msg.data != 'bilinmiyor')

    def _scan_cb(self, msg: LaserScan):
        if not self._engel_aktif:
            return

        ranges = np.array(msg.ranges, dtype=np.float32)
        angles = (msg.angle_min
                  + np.arange(len(ranges), dtype=np.float32) * msg.angle_increment)

        mask = (
            np.isfinite(ranges)
            & (ranges >= ENGEL_MIN_MESAFE)
            & (ranges <= ENGEL_MAX_MESAFE)
            & (angles >= ENGEL_ACI_MIN)
            & (angles <= ENGEL_ACI_MAX)
        )

        r_sel = ranges[mask]
        if len(r_sel) < MIN_NOKTA:
            return

        a_sel = angles[mask]
        xs = r_sel * np.cos(a_sel)
        ys = r_sel * np.sin(a_sel)

        frame = msg.header.frame_id if msg.header.frame_id else 'lidar_link'
        self._pub.publish(_xyz_cloud(xs, ys, frame, msg.header.stamp))

        self.get_logger().debug(
            f'Kayar engel: {len(r_sel)} nokta costmap\'e eklendi',
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
