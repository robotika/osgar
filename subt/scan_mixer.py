"""
  Convert depth image to "lidar" scan
"""
import math

import numpy as np

from osgar.node import Node
from osgar.bus import BusShutdownException
from osgar.lib.depth import depth2dist, FX, CAMW
from osgar.lib.mathex import normalizeAnglePIPI


class ScanMixer(Node):

    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register("scan")
        self.scan = None  # original lidar scan
        self.rs_scan = None  # RealSense RGBD converted scan from ROS
        self.verbose = False

    def update(self):
        channel = super().update()
        assert channel in ["scan", "rs_scan"], channel

        if channel == 'rs_scan':
            self.rs_scan_time = self.time
        elif channel == 'scan':
            assert len(self.scan) in [271, ], len(self.scan)  # Eduro=271
            if self.rs_scan is None and self.slope_scan is None:
                self.publish('scan', self.scan)
                return channel  # when no other sources are available ...

            # TODO some smart processing here
            self.publish('scan', self.scan)
        else:
            assert False, channel  # unsupported channel

        return channel

# vim: expandtab sw=4 ts=4
