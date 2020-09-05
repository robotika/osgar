"""
  Filter sequence of angle readings (from IMU)

  The values are expected to be in -PI/+PI interval (radians)
"""
import math

from osgar.lib.mathex import normalizeAnglePIPI

class AngleFilter:
    def __init__(self, history_size=10):
        self.history_size = history_size
        self.values = []

    def add(self, value):
        self.values.append(value)
        if len(self.values) > self.history_size:
            self.values.pop(0)

    def get(self):
        if len(self.values) == 0:
            return None
        return sum(self.values)/len(self.values)

# vim: expandtab sw=4 ts=4
