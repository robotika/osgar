"""
  This is revised localization from Virtual to be used in System K2 as backup
"""

import collections
import logging
import math
from osgar.node import Node

import osgar.lib.quaternion as quaternion

g_logger = logging.getLogger(__name__)

Pose2d = collections.namedtuple("Pose2d", ("x", "y", "heading"))

class Localization(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        # outputs
        bus.register('pose3d')
        # inputs: origin, orientation, odom
        self.xyz = [0, 0, 0]  # 3D position updated using odometry dist and imu orientation, defined when 'origin' received
        self.orientation = [0, 0, 0, 1]
        self.last_odom = None

    def on_orientation(self, orientation):
        self.orientation = orientation

    def on_odom(self, pose2d):
        x, y, heading = pose2d
        if self.orientation is None:
            return

        odom = Pose2d(x / 1000.0, y / 1000.0, math.radians(heading / 100.0))
        if self.last_odom is not None:
            dist = math.hypot(odom.x - self.last_odom.x, odom.y - self.last_odom.y)
            direction = ((odom.x - self.last_odom.x) * math.cos(self.last_odom.heading) +
                         (odom.y - self.last_odom.y) * math.sin(self.last_odom.heading))
            if direction < 0:
                dist = -dist
        else:
            dist = 0.0

        self.last_odom = odom
        dist3d = quaternion.rotate_vector([dist, 0, 0], self.orientation)
        self.xyz = [a + b for a, b in zip(self.xyz, dist3d)]
        self.bus.publish('pose3d', [self.xyz, self.orientation])

    def update(self):
        channel = super().update()
        handler = getattr(self, "on_" + channel, None)
        if handler is not None:
            handler(getattr(self, channel))
        else:
            assert False, channel  # unsupported channel
        return channel
