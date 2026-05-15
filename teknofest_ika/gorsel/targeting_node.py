#!/usr/bin/env python3
"""
targeting_node.py — Autonomous Shooting: HSV + Hough Circle + PID
Publishes targeting error, status, and turret commands.
"""

import time
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import String
from cv_bridge import CvBridge

from teknofest_ika.utils.pid_controller import PIDController
from teknofest_ika.otonomi.topics import (
    TARGETING_ENABLE_TOPIC, TARGETING_STATUS_TOPIC,
    TARGETING_ERROR_TOPIC, TURRET_CMD_TOPIC,
    CAMERA_PROCESSED_TOPIC, TARGETING_DEBUG_TOPIC,
)


class TargetingNode(Node):
    def __init__(self):
        super().__init__("targeting_node")

        # HSV parameters for target color (default: red target)
        self.declare_parameter("hsv_lower", [0, 100, 100])
        self.declare_parameter("hsv_upper", [10, 255, 255])
        self.declare_parameter("hsv_lower2", [160, 100, 100])  # red wrap-around
        self.declare_parameter("hsv_upper2", [179, 255, 255])

        # Hough Circle parameters
        self.declare_parameter("hough_dp", 1.2)
        self.declare_parameter("hough_min_dist", 50)
        self.declare_parameter("hough_param1", 100)
        self.declare_parameter("hough_param2", 30)
        self.declare_parameter("hough_min_radius", 10)
        self.declare_parameter("hough_max_radius", 200)

        # PID gains
        self.declare_parameter("pid_yaw", [0.5, 0.0, 0.1])
        self.declare_parameter("pid_pitch", [0.5, 0.0, 0.1])
        self.declare_parameter("output_limit", [-45.0, 45.0])  # degrees

        # Alignment & fire logic
        self.declare_parameter("align_threshold_px", 10.0)
        self.declare_parameter("fire_lock_duration_sec", 0.5)
        self.declare_parameter("fire_cooldown_sec", 2.0)
        self.declare_parameter("image_timeout_sec", 0.15)
        self.declare_parameter("target_ema_alpha", 0.4)
        self.declare_parameter("publish_debug", True)

        self.hsv_lower = np.array(self.get_parameter("hsv_lower").value, dtype=np.uint8)
        self.hsv_upper = np.array(self.get_parameter("hsv_upper").value, dtype=np.uint8)
        self.hsv_lower2 = np.array(self.get_parameter("hsv_lower2").value, dtype=np.uint8)
        self.hsv_upper2 = np.array(self.get_parameter("hsv_upper2").value, dtype=np.uint8)

        self.hough_dp = self.get_parameter("hough_dp").value
        self.hough_min_dist = self.get_parameter("hough_min_dist").value
        self.hough_param1 = self.get_parameter("hough_param1").value
        self.hough_param2 = self.get_parameter("hough_param2").value
        self.hough_min_radius = self.get_parameter("hough_min_radius").value
        self.hough_max_radius = self.get_parameter("hough_max_radius").value

        pid_yaw_vals = self.get_parameter("pid_yaw").value
        pid_pitch_vals = self.get_parameter("pid_pitch").value
        out_limit = tuple(self.get_parameter("output_limit").value)

        self.pid_yaw = PIDController(Kp=pid_yaw_vals[0], Ki=pid_yaw_vals[1],
                                     Kd=pid_yaw_vals[2], output_limit=out_limit)
        self.pid_pitch = PIDController(Kp=pid_pitch_vals[0], Ki=pid_pitch_vals[1],
                                       Kd=pid_pitch_vals[2], output_limit=out_limit)

        self.align_threshold = self.get_parameter("align_threshold_px").value
        self.fire_lock_dur = self.get_parameter("fire_lock_duration_sec").value
        self.fire_cooldown = self.get_parameter("fire_cooldown_sec").value
        self.image_timeout = self.get_parameter("image_timeout_sec").value
        self.target_ema_alpha = self.get_parameter("target_ema_alpha").value
        self.publish_debug = self.get_parameter("publish_debug").value

        self.bridge = CvBridge()
        self._status = "STANDBY"
        self._aligned_since = None
        self._search_yaw = 0.0
        self._search_dir = 1.0
        self._search_speed = 5.0  # deg/s
        self._latest_image = None
        self._latest_image_stamp = None
        self._last_processed_stamp = None
        self._target_ema = None  # [cx, cy] smoothed target position
        self._prev_target_found = False
        self._enabled = False      # /targeting/enable ile kontrol edilir
        self._was_enabled = False  # geçiş algılama için

        self.pub_error = self.create_publisher(Point, TARGETING_ERROR_TOPIC, 10)
        self.pub_status = self.create_publisher(String, TARGETING_STATUS_TOPIC, 10)
        self.pub_cmd = self.create_publisher(Vector3, TURRET_CMD_TOPIC, 10)

        if self.publish_debug:
            self.pub_dbg = self.create_publisher(Image, TARGETING_DEBUG_TOPIC, 10)

        self.sub = self.create_subscription(
            Image, CAMERA_PROCESSED_TOPIC,
            self.cb_image, qos_profile_sensor_data)

        # misyon_fsm SHOOT_APPROACH state'i True gönderir, bitince False
        from std_msgs.msg import Bool as BoolMsg
        self.create_subscription(BoolMsg, TARGETING_ENABLE_TOPIC, self._on_enable, 10)

        self.create_timer(0.05, self._control_loop)  # 20 Hz control loop
        self.get_logger().info("TargetingNode başlatıldı — STANDBY (misyon_fsm aktifleştirene kadar bekler)")

    def _on_enable(self, msg):
        if msg.data and not self._enabled:
            self.get_logger().info("TargetingNode: AKTİF — hedef aranıyor")
            self._enabled = True
            self._was_enabled = True
            self.pid_yaw.reset()
            self.pid_pitch.reset()
            self._target_ema = None
            self._aligned_since = None
        elif not msg.data and self._enabled:
            self.get_logger().info("TargetingNode: STANDBY — servo home'a dönüyor")
            self._enabled = False

    def cb_image(self, msg: Image):
        self._latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        # Store ROS 2 timestamp for sync checks (works with both sim and real time)
        self._latest_image_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._latest_image_ros_time = self.get_clock().now()

    def _control_loop(self):
        # Devre dışıysa: geçişte bir kez home gönder, sonra dur
        if not self._enabled:
            if self._was_enabled:
                self.pub_cmd.publish(Vector3(x=0.0, y=0.0, z=0.0))
                self._publish_status("STANDBY")
                self._was_enabled = False
            return

        if self._latest_image is None or self._latest_image_stamp is None:
            self._publish_status("NO_IMAGE")
            return

        # Use ROS 2 clock consistently (supports both sim_time and real time)
        now_ros = self.get_clock().now()
        now = now_ros.nanoseconds / 1e9
        dt = (now_ros - self._latest_image_ros_time).nanoseconds / 1e9

        # Skip stale images
        if dt > self.image_timeout:
            self._publish_status("STALE_IMAGE")
            return

        # Skip already-processed frame
        if self._last_processed_stamp == self._latest_image_stamp:
            return
        self._last_processed_stamp = self._latest_image_stamp

        img = self._latest_image
        h, w = img.shape[:2]
        cx_img = w / 2.0
        cy_img = h / 2.0

        # 1) HSV color filtering
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)
        mask2 = cv2.inRange(hsv, self.hsv_lower2, self.hsv_upper2)
        mask = cv2.bitwise_or(mask1, mask2)

        # Morphological cleanup
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # 2) Hough Circle Transform on mask
        circles = cv2.HoughCircles(
            mask,
            cv2.HOUGH_GRADIENT,
            dp=self.hough_dp,
            minDist=self.hough_min_dist,
            param1=self.hough_param1,
            param2=self.hough_param2,
            minRadius=self.hough_min_radius,
            maxRadius=self.hough_max_radius
        )

        dbg = img.copy() if self.publish_debug else None

        target_cx, target_cy = None, None
        if circles is not None:
            circles = np.round(circles[0, :]).astype(int)
            # Pick largest radius circle
            best = max(circles, key=lambda c: c[2])
            target_cx, target_cy, target_r = best
            if dbg is not None:
                cv2.circle(dbg, (target_cx, target_cy), target_r, (0, 255, 0), 3)
                cv2.circle(dbg, (target_cx, target_cy), 2, (0, 0, 255), 3)

        if target_cx is not None:
            # Temporal EMA smoothing on target position (anti-vibration)
            if self._target_ema is None:
                self._target_ema = np.array([float(target_cx), float(target_cy)])
            else:
                self._target_ema = (
                    self.target_ema_alpha * np.array([target_cx, target_cy]) +
                    (1.0 - self.target_ema_alpha) * self._target_ema
                )
            tx, ty = self._target_ema

            error_x = tx - cx_img
            error_y = ty - cy_img

            yaw_cmd = self.pid_yaw.compute(error_x)
            pitch_cmd = self.pid_pitch.compute(error_y)

            # Check alignment
            aligned = (abs(error_x) < self.align_threshold and
                       abs(error_y) < self.align_threshold)

            if aligned:
                if self._aligned_since is None:
                    self._aligned_since = now
                elif (now - self._aligned_since) >= self.fire_lock_dur:
                    self._status = "ALIGNED"
                else:
                    self._status = "LOCKED"
            else:
                self._aligned_since = None
                self._status = "LOCKED"

            self._prev_target_found = True
            self.pub_error.publish(Point(x=float(error_x), y=float(error_y), z=0.0))
            # z=0.0: ateşleme misyon_fsm ShootState tarafından /shoot_command üzerinden yapılır
            self.pub_cmd.publish(Vector3(x=float(yaw_cmd), y=float(pitch_cmd), z=0.0))

            if dbg is not None:
                cv2.putText(dbg, f"ERR: {error_x:.1f}, {error_y:.1f}", (20, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                cv2.putText(dbg, f"CMD: Y{yaw_cmd:.1f} P{pitch_cmd:.1f} [{self._status}]", (20, 80),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        else:
            # Transition: target lost
            if self._prev_target_found:
                self.pid_yaw.reset()
                self.pid_pitch.reset()
                self._target_ema = None
            self._prev_target_found = False
            self._aligned_since = None
            self._status = "SEARCHING"
            self._search_yaw += self._search_dir * self._search_speed
            if abs(self._search_yaw) > 30.0:
                self._search_dir *= -1.0
            self.pub_cmd.publish(Vector3(x=float(self._search_yaw), y=0.0, z=0.0))
            self.pub_error.publish(Point(x=0.0, y=0.0, z=0.0))
            if dbg is not None:
                cv2.putText(dbg, "SEARCHING...", (20, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)

        self._publish_status(self._status)

        if dbg is not None:
            self.pub_dbg.publish(self.bridge.cv2_to_imgmsg(dbg, "bgr8"))

    def _publish_status(self, status: str):
        self.pub_status.publish(String(data=status))


def main(args=None):
    rclpy.init(args=args)
    node = TargetingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
