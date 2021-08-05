"""
  Extract image from RGBD + robot/camera pose bundle
"""
import math

import numpy as np

from osgar.node import Node
from osgar.lib.depth import decompress as decompress_depth
from osgar.lib import quaternion

from subt.trace import distance3D


DEFAULT_MIN_DIST_STEP = 0.1  # meters
DEFAULT_MIN_ANGLE_STEP = math.radians(10)


class ImageExtractor(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register("image", "depth:gz")
        self.image_sampling = config.get('image_sampling', 1)  # no drop
        self.depth_sampling = config.get('depth_sampling', -1)  # do not log depth
        self.min_dist_step = config.get('min_dist', DEFAULT_MIN_DIST_STEP)
        self.min_angle_step = config.get('min_angle', DEFAULT_MIN_ANGLE_STEP)
        self.index = 0
        self.last_image_anchor_pose = None  # unknown
        self.last_depth_anchor_pose = None  # unknown

    def sufficient_step(self, anchor_pose, robot_pose):
        return (anchor_pose is None or
            distance3D(anchor_pose[0], robot_pose[0]) >= self.min_dist_step or
            quaternion.angle_between(anchor_pose[1], robot_pose[1]) >= self.min_angle_step)

    def on_rgbd(self, data):
        robot_pose, camera_pose, rgb_compressed, depth_compressed = data

        if (self.image_sampling > 0 and self.index % self.image_sampling == 0 and
                self.sufficient_step(self.last_image_anchor_pose, robot_pose)):
            self.publish('image', rgb_compressed)
            self.last_image_anchor_pose = robot_pose

        if (self.depth_sampling > 0 and self.index % self.depth_sampling == 0 and
                self.sufficient_step(self.last_depth_anchor_pose, robot_pose)):
            arr = decompress_depth(depth_compressed) * 1000
            arr = np.clip(arr, 1, 0xFFFF)
            arr = np.ndarray.astype(arr, dtype=np.dtype('H'))
            self.publish('depth', arr)
            self.last_depth_anchor_pose = robot_pose

        self.index += 1

    def update(self):
        channel = super().update()
        handler = getattr(self, "on_" + channel, None)
        if handler is not None:
            handler(getattr(self, channel))
        else:
            assert False, channel  # unsupported channel
        return channel

# vim: expandtab sw=4 ts=4
