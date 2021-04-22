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

    def on_rgbd(self, data):
        robot_pose, camera_pose, rgb_compressed, depth_compressed = data

        arr = decompress_depth(depth_compressed) * 1000
        arr = np.clip(arr, 1, 0xFFFF)
        arr = np.ndarray.astype(arr, dtype=np.dtype('H'))
        self.publish('dist', [int(arr[0][0])])  # TODO center or median

    def update(self):
        channel = super().update()
        handler = getattr(self, "on_" + channel, None)
        if handler is not None:
            handler(getattr(self, channel))
        else:
            assert False, channel  # unsupported channel
        return channel

# vim: expandtab sw=4 ts=4
