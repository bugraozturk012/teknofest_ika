#!/usr/bin/env python3
# Copyright 2026 Buğra Öztürk
# SPDX-License-Identifier: Apache-2.0

"""
tensorrt_inferer.py — TensorRT YOLOv8 Inference Engine for Jetson Orin Nano
Requires: tensorrt, pycuda, numpy, cv2
"""

import os
import time
import ctypes
import numpy as np
import cv2

try:
    import tensorrt as trt
    import pycuda.driver as cuda
    import pycuda.autoinit
    HAS_TRT = True
except ImportError as e:
    print(f"[TensorRTInferer] WARNING: TensorRT/PyCUDA not available: {e}")
    HAS_TRT = False


class TensorRTInferer:
    def __init__(self, engine_path: str, input_shape=(1, 3, 640, 640),
                 conf_thres=0.45, iou_thres=0.45, class_names=None):
        if not HAS_TRT:
            raise RuntimeError("TensorRT or PyCUDA is not installed.")
        if not os.path.isfile(engine_path):
            raise FileNotFoundError(f"TensorRT engine not found: {engine_path}")

        self.engine_path = engine_path
        self.input_shape = input_shape
        self.conf_thres = conf_thres
        self.iou_thres = iou_thres
        self.class_names = class_names or {}

        self.logger = trt.Logger(trt.Logger.WARNING)
        self.runtime = trt.Runtime(self.logger)

        with open(engine_path, "rb") as f:
            self.engine = self.runtime.deserialize_cuda_engine(f.read())

        self.context = self.engine.create_execution_context()
        self.stream = cuda.Stream()

        self.host_inputs = []
        self.cuda_inputs = []
        self.host_outputs = []
        self.cuda_outputs = []
        self.bindings = []

        for i in range(self.engine.num_io_tensors):
            name = self.engine.get_tensor_name(i)
            mode = self.engine.get_tensor_mode(name)
            shape = self.engine.get_tensor_shape(name)
            dtype = trt.nptype(self.engine.get_tensor_dtype(name))
            size = trt.volume(shape) * np.dtype(dtype).itemsize

            if mode == trt.TensorIOMode.INPUT:
                host_mem = cuda.pagelocked_empty(trt.volume(shape), dtype)
                cuda_mem = cuda.mem_alloc(size)
                self.host_inputs.append(host_mem)
                self.cuda_inputs.append(cuda_mem)
            else:
                host_mem = cuda.pagelocked_empty(trt.volume(shape), dtype)
                cuda_mem = cuda.mem_alloc(size)
                self.host_outputs.append(host_mem)
                self.cuda_outputs.append(cuda_mem)

            self.bindings.append(int(cuda_mem))
            self.context.set_tensor_address(name, int(cuda_mem))

        self.context.set_input_shape(self.engine.get_tensor_name(0), self.input_shape)

    def preprocess(self, img_bgr: np.ndarray) -> np.ndarray:
        h, w = img_bgr.shape[:2]
        self.orig_h = h
        self.orig_w = w

        input_h, input_w = self.input_shape[2], self.input_shape[3]
        scale = min(input_w / w, input_h / h)
        nw, nh = int(w * scale), int(h * scale)
        dx, dy = (input_w - nw) // 2, (input_h - nh) // 2

        resized = cv2.resize(img_bgr, (nw, nh), interpolation=cv2.INTER_LINEAR)
        padded = np.full((input_h, input_w, 3), 114, dtype=np.uint8)
        padded[dy:dy + nh, dx:dx + nw, :] = resized

        rgb = padded[:, :, ::-1].astype(np.float32) / 255.0
        chw = np.transpose(rgb, (2, 0, 1))
        return np.expand_dims(chw, axis=0), scale, (dx, dy)

    def infer(self, img_bgr: np.ndarray):
        t0 = time.time()
        input_blob, scale, pad = self.preprocess(img_bgr)
        pre_time = (time.time() - t0) * 1000

        np.copyto(self.host_inputs[0], input_blob.ravel())
        cuda.memcpy_htod_async(self.cuda_inputs[0], self.host_inputs[0], self.stream)

        self.context.execute_async_v3(stream_handle=self.stream.handle)

        for i in range(len(self.host_outputs)):
            cuda.memcpy_dtoh_async(self.host_outputs[i], self.cuda_outputs[i], self.stream)
        self.stream.synchronize()

        inf_time = (time.time() - t0) * 1000 - pre_time

        output = self.host_outputs[0].reshape(self.engine.get_tensor_shape(self.engine.get_tensor_name(1)))
        detections = self.postprocess(output, scale, pad)

        post_time = (time.time() - t0) * 1000 - pre_time - inf_time
        return detections, {"pre": pre_time, "inf": inf_time, "post": post_time}

    def postprocess(self, output, scale, pad):
        # YOLOv8 output shape: (1, 4 + num_classes, num_anchors)
        # e.g. 15 classes -> (1, 19, 8400)
        preds = np.squeeze(output)
        if preds.ndim == 3:
            preds = preds[0]  # drop batch dim -> (channels, anchors)
        num_channels, num_anchors = preds.shape
        num_classes = num_channels - 4

        # Transpose to (anchors, channels)
        preds = preds.T
        scores = np.max(preds[:, 4:], axis=1)
        classes = np.argmax(preds[:, 4:], axis=1)
        mask = scores >= self.conf_thres
        preds = preds[mask]
        scores = scores[mask]
        classes = classes[mask]

        if len(preds) == 0:
            return []

        boxes = preds[:, :4]
        boxes_xyxy = np.zeros_like(boxes)
        boxes_xyxy[:, 0] = boxes[:, 0] - boxes[:, 2] / 2  # x1
        boxes_xyxy[:, 1] = boxes[:, 1] - boxes[:, 3] / 2  # y1
        boxes_xyxy[:, 2] = boxes[:, 0] + boxes[:, 2] / 2  # x2
        boxes_xyxy[:, 3] = boxes[:, 1] + boxes[:, 3] / 2  # y2

        indices = cv2.dnn.NMSBoxes(boxes_xyxy.tolist(), scores.tolist(), self.conf_thres, self.iou_thres)
        if indices is None or len(indices) == 0:
            return []

        # OpenCV versions compatibility: indices may be tuple or ndarray
        if isinstance(indices, tuple):
            indices = np.array(indices)
        indices = indices.flatten()

        results = []
        dx, dy = pad

        for i in indices:
            x1, y1, x2, y2 = boxes_xyxy[i]
            # Remove letterbox padding and rescale to original image
            x1 = (x1 - dx) / scale
            y1 = (y1 - dy) / scale
            x2 = (x2 - dx) / scale
            y2 = (y2 - dy) / scale

            x1 = max(0, min(self.orig_w, x1))
            y1 = max(0, min(self.orig_h, y1))
            x2 = max(0, min(self.orig_w, x2))
            y2 = max(0, min(self.orig_h, y2))

            cls_id = int(classes[i])
            label = self.class_names.get(cls_id, f"class_{cls_id}")
            results.append({
                "label": label,
                "class_id": cls_id,
                "confidence": float(scores[i]),
                "bbox": [float(x1), float(y1), float(x2), float(y2)],
                "center": [float((x1 + x2) / 2), float((y1 + y2) / 2)],
            })
        return results
