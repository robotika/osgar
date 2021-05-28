"""
  Integrate 3 RGBD cameras into 270deg to "lidar" scan
"""
import math

import cv2
import numpy as np

from osgar.node import Node
from osgar.bus import BusShutdownException


class ThreeRGBDToScan(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register("scan")

    def on_rgbd(self, data):
        self.publish('scan', [10000] * 270)

    def update(self):
        channel = super().update()
        handler = getattr(self, "on_" + channel, None)
        if handler is not None:
            handler(getattr(self, channel))
        else:
            assert False, channel  # unsupported channel
        return channel

# vim: expandtab sw=4 ts=4
