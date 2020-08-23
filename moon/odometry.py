"""
  Odometry - handle odometry for Moon rovers
"""
import math

# TODO remove duplicity from moon\vehicles\rover.py !!!
WHEEL_RADIUS = 0.275  # meters
WHEEL_SEPARATION_WIDTH = 1.87325  # meters


class Odometry:
    def __init__(self):
        self.pose2d = 0, 0, 0
        self.prev_position = None

    def update_joint_position(self, names, data):
        self.joint_name = names
        assert self.joint_name is not None
        if self.prev_position is None:
            self.prev_position = data

        diff = [b - a for a, b in zip(self.prev_position, data)]

        assert b'bl_wheel_joint' in self.joint_name, self.joint_name
        # measure odometry from rear wheels
        name = b'bl_wheel_joint'
        name2 = b'br_wheel_joint'
        # name = b'fl_wheel_joint'
        # name2 = b'fr_wheel_joint'
        left = WHEEL_RADIUS * diff[self.joint_name.index(name)]
        right = WHEEL_RADIUS * diff[self.joint_name.index(name2)]
        dist = (left + right)/2
        angle = (right - left)/WHEEL_SEPARATION_WIDTH
        x, y, heading = self.pose2d
        x += math.cos(heading) * dist
        y += math.sin(heading) * dist
        heading += angle
        self.prev_position = data
        self.pose2d = x, y, heading
        return self.pose2d


# vim: expandtab sw=4 ts=4
