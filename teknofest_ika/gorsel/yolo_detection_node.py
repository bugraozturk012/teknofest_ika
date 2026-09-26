#!/usr/bin/env python3
# Copyright 2026 Buğra Öztürk
# SPDX-License-Identifier: Apache-2.0

"""
yolo_detection_node.py — TensorRT YOLOv8 Inference for MAGNESIA LYDIA
Publishes vision_msgs/Detection2DArray
"""

import os
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose, Pose2D, Point2D
from std_msgs.msg import Header

from cv_bridge import CvBridge
from teknofest_ika.utils.tensorrt_inferer import TensorRTInferer, HAS_TRT
from teknofest_ika.otonomi.topics import (
    CAMERA_PROCESSED_TOPIC, YOLO_RAW_TOPIC, YOLO_RAW_DEBUG_TOPIC,
    YOLO_CONFIDENCE_THRESHOLD,
)

try:
    from ultralytics import YOLO as UltralyticsYOLO
    HAS_ULTRALYTICS = True
except ImportError:
    HAS_ULTRALYTICS = False


# Tek kaynak topics.py YOLO_CLASSES (model.names ile eşit tutulur) —
# elle ikinci bir liste tutmak model değişince kaymaya yol açıyordu.
from teknofest_ika.otonomi.topics import YOLO_CLASSES
CLASS_NAMES = {i: ad for i, ad in enumerate(YOLO_CLASSES)}


class YoloDetectionNode(Node):
    def __init__(self):
        super().__init__("yolo_detection_node")

        self.declare_parameter("model_path", "models/best.engine")
        self.declare_parameter("input_shape", [1, 3, 640, 640])
        # topics.py YOLO_CONFIDENCE_THRESHOLD ile uyumlu — yanlış pozitif
        # tabela/koni tespitini azaltmak için resmi eşik kullanılır (Şartname §7).
        self.declare_parameter("conf_thres", YOLO_CONFIDENCE_THRESHOLD)
        self.declare_parameter("iou_thres", 0.45)
        self.declare_parameter("publish_debug_image", True)

        model_path = self.get_parameter("model_path").value
        input_shape = tuple(self.get_parameter("input_shape").value)
        conf_thres = self.get_parameter("conf_thres").value
        iou_thres = self.get_parameter("iou_thres").value
        self.publish_debug = self.get_parameter("publish_debug_image").value

        # Resolve path — os.path.exists() hem dosya hem dizin formatı .pt için çalışır
        if not os.path.exists(model_path):
            try:
                from ament_index_python.packages import get_package_share_directory
                pkg_share = get_package_share_directory('teknofest_ika')
                candidate = os.path.join(pkg_share, model_path)
                if os.path.exists(candidate):
                    model_path = candidate
            except Exception:
                pass
        if not os.path.exists(model_path):
            ws_src = os.path.join(os.path.dirname(__file__), '..', '..', '..')
            candidate = os.path.join(ws_src, model_path)
            if os.path.exists(candidate):
                model_path = candidate

        self._use_ultralytics = False
        self.inferer = None

        # `yolo export format=engine` ULTRALYTICS formatında engine üretir —
        # dosyanın başına metadata ekler. TensorRTInferer ham deserialize
        # yaptığı için bunu açamaz ("magicTag ... incompatible serialization")
        # ve node sessizce yavaş .pt yoluna düşer. Ultralytics kendi engine'ini
        # tanır ve TensorRT arka planıyla koşturur — bu yüzden .engine için
        # ÖNCE ultralytics denenir. Çıkarım arayüzü .pt ile aynı olduğundan
        # aşağıdaki _use_ultralytics yolu değişmeden çalışır.
        if HAS_ULTRALYTICS and model_path.endswith('.engine') and os.path.isfile(model_path):
            self.get_logger().info(f"YOLO (TensorRT engine): {model_path}")
            self._ul_model = UltralyticsYOLO(model_path, task='detect')
            self._ul_conf  = conf_thres
            self._use_ultralytics = True
        elif HAS_TRT and model_path.endswith('.engine') and os.path.isfile(model_path):
            self.get_logger().info(f"Loading TensorRT engine: {model_path}")
            self.inferer = TensorRTInferer(
                engine_path=model_path,
                input_shape=input_shape,
                conf_thres=conf_thres,
                iou_thres=iou_thres,
                class_names=CLASS_NAMES
            )
            self.get_logger().info("TensorRT engine loaded.")
        elif HAS_ULTRALYTICS and os.path.exists(model_path.replace('.engine', '.pt')):
            # Laptop / debug: Ultralytics ile .pt modelini doğrudan çalıştır
            pt_path = model_path.replace('.engine', '.pt')
            self.get_logger().warn(
                f"TensorRT yok — Ultralytics fallback: {pt_path}  (Jetson'da .engine kullan)")
            self._ul_model = UltralyticsYOLO(pt_path)
            self._ul_conf  = conf_thres
            self._use_ultralytics = True
        else:
            self.get_logger().error(
                "Ne TensorRT engine ne de .pt modeli bulunamadı. Node başlatılamıyor.")
            raise RuntimeError("YOLO modeli yüklenemedi")

        self.pub = self.create_publisher(Detection2DArray, YOLO_RAW_TOPIC, 10)
        if self.publish_debug:
            self.pub_dbg = self.create_publisher(Image, YOLO_RAW_DEBUG_TOPIC, 10)

        self.sub = self.create_subscription(
            Image, CAMERA_PROCESSED_TOPIC,
            self.cb_image, qos_profile_sensor_data)

        self.bridge = CvBridge()

    def cb_image(self, msg: Image):
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        if self._use_ultralytics:
            results = self._ul_model(img, conf=self._ul_conf, verbose=False)[0]
            detections = []
            for box in results.boxes:
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                detections.append({
                    "bbox":       (x1, y1, x2, y2),
                    "center":     ((x1 + x2) / 2, (y1 + y2) / 2),
                    "class_id":   int(box.cls[0]),
                    "label":      CLASS_NAMES.get(int(box.cls[0]), str(int(box.cls[0]))),
                    "confidence": float(box.conf[0]),
                })
        else:
            detections, timings = self.inferer.infer(img)
            self.get_logger().debug(
                f"Inference pre={timings['pre']:.1f}ms inf={timings['inf']:.1f}ms "
                f"post={timings['post']:.1f}ms | detections={len(detections)}",
                throttle_duration_sec=2.0)

        det_array = Detection2DArray()
        det_array.header = msg.header

        dbg = img.copy() if self.publish_debug else None

        for d in detections:
            det = Detection2D()
            det.header = msg.header
            x1, y1, x2, y2 = d["bbox"]
            cx, cy = d["center"]
            w = x2 - x1
            h = y2 - y1

            det.bbox.center = Pose2D(position=Point2D(x=float(cx), y=float(cy)), theta=0.0)
            det.bbox.size_x = float(w)
            det.bbox.size_y = float(h)

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = str(d["class_id"])
            hyp.hypothesis.score = d["confidence"]
            det.results.append(hyp)

            det_array.detections.append(det)

            if dbg is not None:
                cv2.rectangle(dbg, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                label = f"{d['label']} {d['confidence']:.2f}"
                cv2.putText(dbg, label, (int(x1), max(20, int(y1) - 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        self.pub.publish(det_array)

        if dbg is not None:
            self.pub_dbg.publish(self.bridge.cv2_to_imgmsg(dbg, "bgr8"))


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
