"""
  Filter sequence of angle readings (from IMU)

  The values are expected to be in -PI/+PI interval (radians)
"""
import math

from osgar.lib.mathex import normalizeAnglePIPI

class AngleFilter:
    def __init__(self, history_size=10):
        self.history_size = history_size
        self.values_x, self.values_y = [], []

    def add(self, value):
        self.values_x.append(math.cos(value))
        self.values_y.append(math.sin(value))
        if len(self.values_x) > self.history_size:
            self.values_x.pop(0)
            self.values_y.pop(0)

    def get(self):
        if len(self.values_x) == 0:
            return None
        return math.atan2(sum(self.values_y), sum(self.values_x))

# vim: expandtab sw=4 ts=4
