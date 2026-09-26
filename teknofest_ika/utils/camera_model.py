#!/usr/bin/env python3
# Copyright 2026 Buğra Öztürk
# SPDX-License-Identifier: Apache-2.0

"""
camera_model.py — PinholeCameraModel wrapper for 3D ray projection
"""

import numpy as np
from image_geometry import PinholeCameraModel
from sensor_msgs.msg import CameraInfo


class CameraModelHelper:
    def __init__(self):
        self._model = PinholeCameraModel()
        self._ready = False

    def update_camera_info(self, msg: CameraInfo):
        self._model.fromCameraInfo(msg)
        self._ready = True

    def is_ready(self) -> bool:
        return self._ready

    def project_pixel_to_3d_ray(self, pixel_x: float, pixel_y: float) -> np.ndarray:
        if not self._ready:
            raise RuntimeError("CameraModelHelper: CameraInfo not set yet")
        return self._model.projectPixelTo3dRay((pixel_x, pixel_y))

    def get_camera_intrinsics(self):
        if not self._ready:
            return None
        return {
            'fx': self._model.fx(),
            'fy': self._model.fy(),
            'cx': self._model.cx(),
            'cy': self._model.cy(),
            'width': self._model.width,
            'height': self._model.height,
        }
