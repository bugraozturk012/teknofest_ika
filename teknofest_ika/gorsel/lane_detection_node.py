#!/usr/bin/env python3
# Copyright 2026 Buğra Öztürk
# SPDX-License-Identifier: Apache-2.0

"""
lane_detection_node.py — Classical Lane Detection using Hough Lines + Bird's Eye View
Publishes lane center offset (std_msgs/Float64)

Bu düğümün sistemde tüketicisi yok: `/lane/center_offset`, `/lane/center_offset_m`
ve `/lane/debug` topic'lerine hiçbir node abone değil, `scripts/lydia_startup.sh`
onu başlatmıyor ve hiçbir launch dosyasında geçmiyor. `setup.py`'de kayıtlı
olduğu için `ros2 run` ile elle çalıştırılabilir. Şerit takibi bir istasyon
gereksinimi haline gelirse çıkışının bir tüketiciye bağlanması gerekir.
"""

import math
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from cv_bridge import CvBridge
from teknofest_ika.otonomi.topics import CAMERA_PROCESSED_TOPIC


class LaneDetectionNode(Node):
    def __init__(self):
        super().__init__("lane_detection_node")

        self.declare_parameter("canny_low", 50)
        self.declare_parameter("canny_high", 150)
        self.declare_parameter("hough_rho", 1)
        self.declare_parameter("hough_theta", 1)  # degrees
        self.declare_parameter("hough_threshold", 50)
        self.declare_parameter("hough_min_line_length", 40)
        self.declare_parameter("hough_max_line_gap", 20)
        self.declare_parameter("roi_top_ratio", 0.6)   # ROI starts at 60% height
        self.declare_parameter("camera_height_m", 0.5)  # Camera height above ground
        self.declare_parameter("camera_hfov_deg", 60.0) # Horizontal FOV
        self.declare_parameter("publish_debug", True)

        self.canny_low = self.get_parameter("canny_low").value
        self.canny_high = self.get_parameter("canny_high").value
        self.hough_rho = self.get_parameter("hough_rho").value
        self.hough_theta = math.radians(self.get_parameter("hough_theta").value)
        self.hough_threshold = self.get_parameter("hough_threshold").value
        self.hough_min_line_length = self.get_parameter("hough_min_line_length").value
        self.hough_max_line_gap = self.get_parameter("hough_max_line_gap").value
        self.roi_top = self.get_parameter("roi_top_ratio").value
        self.camera_height = self.get_parameter("camera_height_m").value
        self.camera_hfov = math.radians(self.get_parameter("camera_hfov_deg").value)
        self.publish_debug = self.get_parameter("publish_debug").value

        self.bridge = CvBridge()
        self.pub = self.create_publisher(Float64, "/lane/center_offset", 10)
        self.pub_m = self.create_publisher(Float64, "/lane/center_offset_m", 10)
        if self.publish_debug:
            self.pub_dbg = self.create_publisher(Image, "/lane/debug", 10)

        self.sub = self.create_subscription(
            Image, CAMERA_PROCESSED_TOPIC,
            self.cb_image, qos_profile_sensor_data)

        self.get_logger().info("LaneDetectionNode started.")

    def cb_image(self, msg: Image):
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        h, w = img.shape[:2]

        # 1) Grayscale + Gaussian
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)

        # 2) Canny
        edges = cv2.Canny(blur, self.canny_low, self.canny_high)

        # 3) ROI mask (trapezoid focusing bottom-center)
        roi = np.zeros_like(edges)
        top_y = int(h * self.roi_top)
        pts = np.array([
            [int(w * 0.1), h],
            [int(w * 0.45), top_y],
            [int(w * 0.55), top_y],
            [int(w * 0.9), h]
        ], np.int32)
        cv2.fillPoly(roi, [pts], 255)
        masked = cv2.bitwise_and(edges, roi)

        # 4) Hough Lines
        lines = cv2.HoughLinesP(
            masked,
            self.hough_rho,
            self.hough_theta,
            self.hough_threshold,
            minLineLength=self.hough_min_line_length,
            maxLineGap=self.hough_max_line_gap
        )

        offset = 0.0
        if lines is not None and len(lines) > 0:
            left_lines = []
            right_lines = []
            for line in lines:
                x1, y1, x2, y2 = line[0]
                if x2 == x1:
                    continue
                slope = (y2 - y1) / float(x2 - x1)
                # Filter out horizontal-ish lines
                if abs(slope) < 0.3:
                    continue
                if slope < 0:
                    left_lines.append((slope, y1 - slope * x1))
                else:
                    right_lines.append((slope, y1 - slope * x1))

            left_fit = None
            right_fit = None
            if len(left_lines) > 0:
                left_fit = np.mean(left_lines, axis=0)
            if len(right_lines) > 0:
                right_fit = np.mean(right_lines, axis=0)

            # Compute lane center at bottom of image (y = h)
            y_eval = h
            left_x = None
            right_x = None
            if left_fit is not None:
                left_x = int((y_eval - left_fit[1]) / left_fit[0])
            if right_fit is not None:
                right_x = int((y_eval - right_fit[1]) / right_fit[0])

            if left_x is not None and right_x is not None:
                lane_center = (left_x + right_x) / 2.0
            elif left_x is not None:
                lane_center = left_x + w * 0.25
            elif right_x is not None:
                lane_center = right_x - w * 0.25
            else:
                lane_center = w / 2.0

            img_center = w / 2.0
            offset = lane_center - img_center

            if self.publish_debug:
                # Draw lines
                for line in lines:
                    x1, y1, x2, y2 = line[0]
                    cv2.line(img, (x1, y1), (x2, y2), (0, 0, 255), 2)
                if left_x is not None:
                    cv2.circle(img, (left_x, y_eval), 5, (255, 0, 0), -1)
                if right_x is not None:
                    cv2.circle(img, (right_x, y_eval), 5, (255, 0, 0), -1)
                cv2.circle(img, (int(lane_center), y_eval), 7, (0, 255, 255), -1)
                cv2.circle(img, (int(img_center), y_eval), 7, (255, 255, 0), -1)
                cv2.putText(img, f"offset: {offset:.1f}px", (20, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)

        self.pub.publish(Float64(data=float(offset)))

        # Approximate pixel-to-meter conversion at image bottom (near field)
        # width_m = 2 * h * tan(fov/2)
        if w > 0:
            width_m = 2.0 * self.camera_height * math.tan(self.camera_hfov / 2.0)
            meters_per_pixel = width_m / w
            offset_m = offset * meters_per_pixel
            self.pub_m.publish(Float64(data=float(offset_m)))
            if self.publish_debug:
                cv2.putText(img, f"offset: {offset_m:.2f}m", (20, 80),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 2)

        if self.publish_debug:
            self.pub_dbg.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))


def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
