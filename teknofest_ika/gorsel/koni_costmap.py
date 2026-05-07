#!/usr/bin/env python3
"""
koni_costmap.py — Trafik Konisi → Nav2 Costmap Köprüsü
=======================================================
Görüntü ekibinden gelen /cone_positions (PoseArray) topic'ini
Nav2 ObstacleLayer'ın tüketebileceği sensor_msgs/PointCloud2
formatına çevirir.

nav2_params.yaml'da obstacle_layer'a şu kaynak eklenmiştir:
  cone_cloud:
    topic: /cone_cloud
    data_type: "PointCloud2"
    marking: true
    clearing: false

Görüntü ekibi koni pozisyonlarını map frame'inde yayınlamalıdır:
  ros2 topic pub /cone_positions geometry_msgs/msg/PoseArray \
    '{header: {frame_id: "map"}, poses: [{position: {x: 1.0, y: 2.0, z: 0.0}}]}' --once

GİRİŞ : /cone_positions (geometry_msgs/PoseArray)  — görüntü ekibi
ÇIKIŞ : /cone_cloud     (sensor_msgs/PointCloud2)   — Nav2 ObstacleLayer
"""
import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import PointCloud2, PointField


def _build_cloud(poses, frame_id: str, stamp) -> PointCloud2:
    """PoseArray → XYZ PointCloud2 (12 byte/nokta: float32 x,y,z)."""
    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    step = 12
    data = bytearray()
    for p in poses:
        data += struct.pack('fff',
                            float(p.position.x),
                            float(p.position.y),
                            0.0)   # z=0: 2D costmap zemin seviyesi

    cloud = PointCloud2()
    cloud.header.stamp    = stamp
    cloud.header.frame_id = frame_id
    cloud.height          = 1
    cloud.width           = len(poses)
    cloud.fields          = fields
    cloud.is_bigendian    = False
    cloud.point_step      = step
    cloud.row_step        = step * len(poses)
    cloud.data            = bytes(data)
    cloud.is_dense        = True
    return cloud


class KoniCostmap(Node):

    def __init__(self):
        super().__init__('koni_costmap')

        self.declare_parameter('map_frame', 'map')
        self._frame = self.get_parameter('map_frame').value

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.create_subscription(
            PoseArray, '/cone_positions', self._cb, qos)

        self._pub = self.create_publisher(PointCloud2, '/cone_cloud', 10)

        self.get_logger().info(
            f'KoniCostmap hazır | '
            f'/cone_positions (PoseArray) → /cone_cloud (PointCloud2) | '
            f'frame={self._frame}'
        )

    def _cb(self, msg: PoseArray):
        if not msg.poses:
            return

        frame = msg.header.frame_id if msg.header.frame_id else self._frame
        cloud = _build_cloud(msg.poses, frame, msg.header.stamp)
        self._pub.publish(cloud)

        self.get_logger().debug(
            f'{len(msg.poses)} koni costmap\'e eklendi',
            throttle_duration_sec=2.0
        )


def main(args=None):
    rclpy.init(args=args)
    node = KoniCostmap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
