"""
  Integrate 3 RGBD cameras into 270deg to "lidar" scan
"""
import math

import cv2
import numpy as np

from osgar.node import Node
from osgar.bus import BusShutdownException
from osgar.lib.depth import decompress as decompress_depth


class ThreeRGBDToScan(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register("scan")
        self.center = [0] * 640
        self.left = [0] * 360
        self.right = [0] * 360

    def on_rgbd(self, data):
        robot_pose, camera_pose, rgb_compressed, depth_compressed = data

        arr = decompress_depth(depth_compressed) * 1000
        arr = np.clip(arr, 1, 0xFFFF)
        arr = np.ndarray.astype(arr, dtype=np.dtype('H'))
        assert arr.shape == (360, 640), arr.shape

        offset_y = camera_pose[0][1]
        if abs(offset_y) < 0.0001:
            self.center = arr[180].tolist()
        elif offset_y > 0:
            self.left = arr.T[320].tolist()
        else:
            self.right = arr.T[320].tolist()

        self.publish('scan', self.left + self.center + self.right)

    def update(self):
        channel = super().update()
        handler = getattr(self, "on_" + channel, None)
        if handler is not None:
            handler(getattr(self, channel))
        else:
            assert False, channel  # unsupported channel
        return channel

# vim: expandtab sw=4 ts=4
