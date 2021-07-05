"""
  Extract distance from RGBD and report it as single lidar distance
"""
import numpy as np

from osgar.node import Node
from osgar.bus import BusShutdownException
from osgar.lib.depth import decompress as decompress_depth


class RGBD2Dist(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register("dist")
        self.verbose = False

    def on_rgbd(self, data):
        robot_pose, camera_pose, rgb_compressed, depth_compressed = data

        arr = decompress_depth(depth_compressed)
        height, width = arr.shape
        dist = arr[height//2][width//2]
        if dist == float('inf'):
            # out of range, but maybe mistake in RGBD simulation model
            # unfortunately it is the same value for down and up camera
            dist = arr[height//2:, :].min()  # up pointing camera will see some walls
                                             # but there is also Velodyne cooler!
            if dist == float('inf'):
                # everything is "black" -> down pointing camera
                dist = 0.1
        if dist == float('-inf'):
            # blind
            dist = 0.1
        if self.verbose:
            print(dist)
        self.publish('dist', [float(dist)])

    def update(self):
        channel = super().update()
        handler = getattr(self, "on_" + channel, None)
        if handler is not None:
            handler(getattr(self, channel))
        else:
            assert False, channel  # unsupported channel
        return channel

# vim: expandtab sw=4 ts=4
