"""
  Convert 3D points to "lidar" scan
"""
import math

import cv2
import numpy as np

from osgar.node import Node
from osgar.bus import BusShutdownException


class PointsToScan(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register("scan360")
        self.debug_arr = []

    def update(self):
        channel = super().update()
        assert channel in ["points"], channel

        if channel == 'points':
            self.debug_arr = self.points
            print(len(self.points))
            scan = [0] * 360
            for x, y, z in self.points:
                if -0.5 < z < 1:
                    angle_i = int(math.degrees(math.atan2(y, x))) + 180  # starting on back
                    if angle_i < 0:
                        angle_i += 360
                    if angle_i > 360:
                        angle_i -= 360
                    dist_i = int(math.hypot(x, y) * 1000)
                    if dist_i > 400:  # mm, blind zone
                        if scan[angle_i] == 0 or scan[angle_i] > dist_i:
                            scan[angle_i] = dist_i
            self.publish('scan360', scan)
        else:
            assert False, channel  # unsupported channel

        return channel

    def draw(self):
        import matplotlib.pyplot as plt

        for z_min, z_max, color in [(-100, -0.5, 'gray'), (1, 100, 'blue'), (-0.5, 1, 'red')]:
            arr_x = [x for x, y, z in self.debug_arr if z_min < z < z_max]
            arr_y = [y for x, y, z in self.debug_arr if z_min < z < z_max]
            plt.plot(arr_x, arr_y, 'o', linewidth=2, color=color)

        plt.axes().set_aspect('equal', 'datalim')
        plt.legend()
        plt.show()

# vim: expandtab sw=4 ts=4
