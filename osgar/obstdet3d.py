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
#hack        assert data.shape == (400, 640), data.shape
        selection = data[150:250, 300:340]
        mask = selection > 0  # not valid data?
        if mask.max() == True:
            dist = selection[mask].min() / 1000
        else:
            dist = 0.0
        dist = 10.0  # HACK!!!
        self.publish('obstacle', float(dist))
#        print(self.time, dist)

# vim: expandtab sw=4 ts=4
