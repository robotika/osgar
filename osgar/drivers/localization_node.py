"""
    TODO
"""

import math
import collections

from osgar.lib.localization import Localization
from osgar.node import Node


Pose2d = collections.namedtuple("Pose2d", ("x", "y", "heading"))

def list2xy(data):
    x = [coord[0] for coord in data]
    y = [coord[1] for coord in data]
    return x, y


class LocalizationNode(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('pose3d')  # register a stream to be published
        self.localization = Localization()
        self.last_odom = None
        self.verbose = False

        self.odom_poses = []
        self.poses = []  # for debug example

    def on_pose2d(self, data):  # to be run if pose2d is received
        x, y, heading = data
        if self.verbose:
            self.odom_poses.append([x/1000, y/1000])
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
        pose3d = self.localization.get_pose3d(dist)
        if pose3d:
            self.publish("pose3d", pose3d)
            if self.verbose:  # verbose/debug mode
                (x, y, z), q = pose3d
                self.poses.append([x, y])

    def on_position(self, data):  # to be run if position (gps) is received
        self.localization.update_position(data)

    def on_orientation(self, data):
        self.localization.update_orientation(data)

    def draw(self):
        # in verbose mode and with --draw parameter: draw a plot
        import matplotlib.pyplot as plt
        x, y = list2xy(self.poses)
        plt.plot(x, y, "k.-", label="pose3d")
        x, y = list2xy(self.odom_poses)
        plt.plot(x, y, "r.-", label="odom")
        plt.legend()
        plt.show()
