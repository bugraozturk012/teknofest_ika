"""
scan_relay.py — Lidar Scan Timestamp ve Frame Düzeltici
========================================================
YDLidar iç hata [0x202] durumunda scan'lere timestamp=0 ve
frame_id='laser_frame' atıyor. SLAM bu scan'leri işleyemiyor.

Bu node:
  - /scan_raw dinler (lidar çıkışı)
  - timestamp=0 ise sistem saatini yazar
  - frame_id'yi 'lidar_link' olarak düzeltir
  - /scan yayınlar (SLAM girişi)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.clock import Clock, ClockType
from sensor_msgs.msg import LaserScan

_BEST_EFFORT = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


class ScanRelay(Node):

    def __init__(self):
        super().__init__('scan_relay')
        self._wall = Clock(clock_type=ClockType.SYSTEM_TIME)
        self._sub = self.create_subscription(
            LaserScan, '/scan_raw', self._cb, _BEST_EFFORT)
        self._pub = self.create_publisher(LaserScan, '/scan_lidar', 10)
        self.get_logger().info('ScanRelay hazır: /scan_raw → /scan_lidar')

    def _cb(self, msg: LaserScan):
        msg.header.stamp = self._wall.now().to_msg()
        msg.header.frame_id = 'lidar_link'
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
