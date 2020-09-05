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


class AngleFilterIMU:
    """
      Convenience IMU filter for all 3 angles
    """
    def __init__(self, history_size=10):
        self.yaw = AngleFilter(history_size=history_size)
        self.pitch = AngleFilter(history_size=history_size)
        self.roll = AngleFilter(history_size=history_size)

    def add(self, yaw, pitch, roll):
        self.yaw.add(yaw)
        self.pitch.add(pitch)
        self.roll.add(roll)

    def get(self):
        return self.yaw.get(), self.pitch.get(), self.roll.get()

# vim: expandtab sw=4 ts=4
