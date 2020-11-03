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

    def on_points(self, data):
        assert data.shape == (16, 1000, 3), data.shape
        self.debug_arr = data
        index = (np.arange(360) * (1000/360)).astype(int)
        xyz = data[8][index]  # mid index for 16 lidars
        X, Y, Z = xyz[:, 0], xyz[:, 1], xyz[:, 2]
        dist_i = (np.hypot(X, Y) * 1000).astype(int)
        mask = dist_i < 400
        dist_i[mask] = 0
        self.publish('scan360', dist_i.tolist())

    def update(self):
        channel = super().update()
        handler = getattr(self, "on_" + channel, None)
        if handler is not None:
            handler(getattr(self, channel))
        else:
            assert False, channel  # unsupported channel
        return channel

    def draw(self):
        import matplotlib.pyplot as plt

        data = self.debug_arr.reshape(16 * 1000, 3)
        for z_min, z_max, color in [(-100, -0.5, 'gray'), (1, 100, 'blue'), (-0.5, 1, 'red')]:
            arr_x = [x for x, y, z in data if z_min < z < z_max]
            arr_y = [y for x, y, z in data if z_min < z < z_max]
            plt.plot(arr_x, arr_y, 'o', linewidth=2, color=color)

        plt.axes().set_aspect('equal', 'datalim')
        plt.legend()
        plt.show()

# vim: expandtab sw=4 ts=4
