#!/usr/bin/env python3
"""
map_image_node.py — SLAM haritasını (OccupancyGrid) GCS dashboard için görüntüye çevirir

Girdi : /map        (nav_msgs/OccupancyGrid)  — slam_toolbox
Çıktı : /map/image  (sensor_msgs/Image, bgr8)  — ika_dashboard.py 'SLAM Haritası' paneli

Piksel eşlemesi: -1 (bilinmeyen) → gri, 0 (boş) → beyaz, 100 (dolu) → siyah,
aradaki değerler doluluk olasılığına göre doğrusal griye ölçeklenir.
Görüntünün ilk satırı üstte olacak şekilde dikey çevrilir (OccupancyGrid
satır 0 haritanın altını temsil eder, görüntüde ise üstü temsil eder).
"""

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from teknofest_ika.otonomi.topics import MAP_TOPIC, MAP_IMAGE_TOPIC

BILINMEYEN_GRI = 127


class MapImageNode(Node):

    def __init__(self):
        super().__init__("map_image_node")
        self._bridge = CvBridge()

        # slam_toolbox /map'i transient_local+reliable yayınlıyor (latched harita)
        harita_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._pub = self.create_publisher(Image, MAP_IMAGE_TOPIC, qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, MAP_TOPIC, self._on_map, harita_qos)

        self.get_logger().info(f"MapImageNode hazır | {MAP_TOPIC} → {MAP_IMAGE_TOPIC}")

    def _on_map(self, msg: OccupancyGrid):
        w, h = msg.info.width, msg.info.height
        if w == 0 or h == 0:
            return

        veri = np.array(msg.data, dtype=np.int16).reshape(h, w)
        gri = np.full((h, w), BILINMEYEN_GRI, dtype=np.uint8)

        bilinen = veri >= 0
        gri[bilinen] = (255 - (veri[bilinen] * 255 // 100)).astype(np.uint8)

        gri = np.flipud(gri)
        renkli = cv2.cvtColor(gri, cv2.COLOR_GRAY2BGR)

        img_msg = self._bridge.cv2_to_imgmsg(renkli, encoding="bgr8")
        img_msg.header = msg.header
        self._pub.publish(img_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MapImageNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
