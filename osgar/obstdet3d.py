"""
  Obstacle Detection 3D
"""
import numpy as np

from osgar.node import Node


class ObstacleDetector3D(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('obstacle')

    def on_depth(self, data):
        assert data.shape == (400, 640), data.shape
        selection = data[150:250, 300:340]
        mask = selection > 0  # not valid data?
        dist = selection[mask].min() / 1000
        self.publish('obstacle', dist)
#        print(self.time, dist)

# vim: expandtab sw=4 ts=4
