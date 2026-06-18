#!/usr/bin/env python3
"""
cone_fusion_node.py — Cone Detection + LiDAR Fusion + Nav2 Costmap LETHAL PointCloud2
Uses message_filters.ApproximateTimeSynchronizer
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

import message_filters
from sensor_msgs.msg import LaserScan, PointCloud2, PointField, CameraInfo
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import Header
from image_geometry import PinholeCameraModel

from teknofest_ika.otonomi.topics import SCAN_FILTERED_TOPIC, YOLO_RAW_TOPIC, CONE_FUSION_CLOUD_TOPIC


class ConeFusionNode(Node):
    def __init__(self):
        super().__init__("cone_fusion_node")

        self.declare_parameter("camera_fov_deg", 60.0)
        self.declare_parameter("image_width", 640)
        self.declare_parameter("cone_safety_radius_m", 0.4)
        self.declare_parameter("cone_min_confidence", 0.45)
        self.declare_parameter("lidar_window", 8)
        # yolo_detection_node class_id'yi integer string olarak yayınlar: str(13) = "13"
        # "trafik_huni" string karşılaştırması YANLIŞ — hiç eşleşmez.
        self.declare_parameter("target_label", "15")
        self.declare_parameter("safety_sphere_points", 20)

        self.fov_deg = self.get_parameter("camera_fov_deg").value
        self.image_width = self.get_parameter("image_width").value
        self.safety_radius = self.get_parameter("cone_safety_radius_m").value
        self.min_conf = self.get_parameter("cone_min_confidence").value
        self.lidar_window = self.get_parameter("lidar_window").value
        self.target_label = self.get_parameter("target_label").value
        self.safety_sphere_points = self.get_parameter("safety_sphere_points").value

        self.pub = self.create_publisher(PointCloud2, CONE_FUSION_CLOUD_TOPIC, 10)

        # Camera model for accurate pixel-to-ray projection
        self._camera_model = PinholeCameraModel()
        self._camera_info_ready = False

        # Cache CameraInfo for time-sync lookups
        self._camera_info_cache = message_filters.Cache(
            message_filters.Subscriber(self, CameraInfo, "/ileri_kamera/camera_info"),
            cache_size=10,
            allow_headerless=False
        )

        # Subscribers with ApproximateTimeSynchronizer
        det_sub  = message_filters.Subscriber(self, Detection2DArray, YOLO_RAW_TOPIC)
        scan_sub = message_filters.Subscriber(self, LaserScan, SCAN_FILTERED_TOPIC)

        self.sync = message_filters.ApproximateTimeSynchronizer(
            [det_sub, scan_sub],
            queue_size=10,
            slop=0.05
        )
        self.sync.registerCallback(self.cb_fusion)

        self.get_logger().info("ConeFusionNode started (ApproximateTime sync).")

    def _lookup_camera_info(self, target_stamp):
        """Find the CameraInfo closest to target timestamp."""
        if self._camera_info_cache.getOldestTime() is None:
            return None
        # message_filters.Cache API: getElemBeforeTime returns closest msg before target
        msg = self._camera_info_cache.getElemBeforeTime(target_stamp)
        if msg is None:
            # Fallback: get the first msg after target
            msg = self._camera_info_cache.getElemAfterTime(target_stamp)
        return msg

    def cb_fusion(self, det_msg: Detection2DArray, scan_msg: LaserScan):
        cone_centers = []

        # Try to get time-synced CameraInfo for accurate projection
        cam_info = self._lookup_camera_info(det_msg.header.stamp)
        if cam_info is not None and not self._camera_info_ready:
            self._camera_model.fromCameraInfo(cam_info)
            self._camera_info_ready = True
            self.get_logger().info(
                f"CameraInfo synced: fx={self._camera_model.fx():.1f} "
                f"cx={self._camera_model.cx():.1f}")

        for det in det_msg.detections:
            if not det.results:
                continue
            best = max(det.results, key=lambda r: r.hypothesis.score)
            label = best.hypothesis.class_id
            conf = best.hypothesis.score

            if label != self.target_label or conf < self.min_conf:
                continue

            cx = det.bbox.center.x
            # Use configured image width; detection bbox center is in pixel coords
            img_width = float(self.image_width)

            angle_deg = self._pixel_to_angle(cx, img_width)
            dist = self._get_lidar_distance(scan_msg, angle_deg)

            if dist is None:
                continue

            # Convert to base_link 2D coords (x forward, y left)
            angle_rad = math.radians(angle_deg)
            x = dist * math.cos(angle_rad)
            y = dist * math.sin(angle_rad)

            cone_centers.append([x, y, 0.0])
            self.get_logger().debug(
                f"Cone: angle={angle_deg:.1f}deg dist={dist:.2f}m xyz=({x:.2f},{y:.2f},0)")

        if not cone_centers:
            return

        # Generate safety sphere points around each cone center
        all_points = []
        for center in cone_centers:
            all_points.extend(self._generate_safety_sphere(center))

        points = np.array(all_points, dtype=np.float32)

        # Noktalar base_footprint koordinat sisteminde hesaplandı (x ileri, y sol).
        # det_msg.header kamera frame'i taşır — Nav2'nin costmap'e doğru yerleştirmesi
        # için frame_id 'base_footprint' olarak üzerine yazılır.
        base_header = Header()
        base_header.stamp = det_msg.header.stamp
        base_header.frame_id = 'base_footprint'
        cloud = self._create_cloud(base_header, points)
        self.pub.publish(cloud)

    def _generate_safety_sphere(self, center, z_levels=3):
        """Generate a dome of points around the cone center for Nav2 lethal marking."""
        points = []
        r = self.safety_radius
        cx, cy, cz = center
        # Hemisphere above ground
        for phi in np.linspace(0, math.pi / 2, z_levels):
            ring_points = max(3, int(self.safety_sphere_points * math.cos(phi)))
            for theta in np.linspace(0, 2 * math.pi, ring_points, endpoint=False):
                px = cx + r * math.cos(theta) * math.cos(phi)
                py = cy + r * math.sin(theta) * math.cos(phi)
                pz = cz + r * math.sin(phi)
                points.append([px, py, pz])
        # Include center point itself
        points.append([cx, cy, cz])
        return points

    def _pixel_to_angle(self, pixel_x: float, image_width: float) -> float:
        if self._camera_info_ready:
            # Use calibrated camera model for accurate ray angle
            ray = self._camera_model.projectPixelTo3dRay((pixel_x, self._camera_model.cy()))
            # ray is a unit vector (x, y, z) in camera frame where z is forward
            angle_rad = math.atan2(ray[0], ray[2])  # atan2(x, z) gives yaw angle
            return math.degrees(angle_rad)
        else:
            # Fallback to simple linear approximation
            center_x = image_width / 2.0
            dx = pixel_x - center_x
            angle_per_pixel = self.fov_deg / image_width
            return dx * angle_per_pixel

    def _get_lidar_distance(self, scan: LaserScan, angle_deg: float):
        angle_rad = math.radians(angle_deg)
        if angle_rad < scan.angle_min or angle_rad > scan.angle_max:
            return None
        idx = int((angle_rad - scan.angle_min) / scan.angle_increment)
        w = self.lidar_window
        start = max(0, idx - w)
        end = min(len(scan.ranges), idx + w + 1)
        vals = [r for r in scan.ranges[start:end]
                if math.isfinite(r) and scan.range_min < r < scan.range_max]
        if not vals:
            return None
        return float(np.median(vals))

    def _create_cloud(self, header: Header, points: np.ndarray):
        cloud = PointCloud2()
        cloud.header = header
        cloud.height = 1
        cloud.width = len(points)
        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.point_step = 12
        cloud.row_step = cloud.point_step * cloud.width
        cloud.is_bigendian = False
        cloud.is_dense = True
        cloud.data = points.tobytes()
        return cloud


def main(args=None):
    rclpy.init(args=args)
    node = ConeFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
