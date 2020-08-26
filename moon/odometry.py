"""
  Odometry - handle odometry for Moon rovers
"""
import math

# TODO remove duplicity from moon\vehicles\rover.py !!!
WHEEL_RADIUS = 0.275  # meters
WHEEL_SEPARATION_WIDTH = 1.87325  # meters
WHEEL_SEPARATION_LENGTH = 1.5748  # meters

WHEEL_NAMES = [b'fl', b'fr', b'bl', b'br']


class Odometry:
    def __init__(self):
        self.pose2d = 0, 0, 0
        self.prev_position = None
        self.crab_limit = math.radians(5)
        self.turn_in_place_limit = math.radians(30)
        self.debug_arr = []

    def is_crab_step(self, steering):
        avg = sum(steering)/4
        offset = max([abs(s - avg) for s in steering])
        return offset < self.crab_limit

    def is_turn_in_place(self, steering):
        fl, fr, bl, br = steering
        lim = self.turn_in_place_limit
        return fl < -lim and fr > lim and bl > lim and br < -lim

    def update_joint_position(self, names, data, verbose=False):
        self.joint_name = names
        assert self.joint_name is not None
        if self.prev_position is None:
            self.prev_position = data

        diff = [b - a for a, b in zip(self.prev_position, data)]
        steering = [data[names.index(n + b'_steering_arm_joint')]
                    for n in WHEEL_NAMES]
        if verbose:
            self.debug_arr.append(steering)
        x, y, heading = self.pose2d
        drive = [diff[names.index(n + b'_wheel_joint')]
                 for n in WHEEL_NAMES]
        if self.is_crab_step(steering):
            dist = WHEEL_RADIUS * sum(drive)/4
            # TODO use rather abs min distance with sign
            angle = sum(steering)/4  # all wheels point the same direction
            x += math.cos(heading + angle) * dist
            y += math.sin(heading + angle) * dist
            # expected no change of heading
        elif self.is_turn_in_place(steering):
            # not expected change in x and y coordinates
            # average distance driven on wheel on the circle
            dist = WHEEL_RADIUS * (-drive[0] + drive[1] - drive[2] + drive[3])/4
            radius = math.hypot(WHEEL_SEPARATION_WIDTH/2, WHEEL_SEPARATION_LENGTH/2)
            heading += dist/radius
        else:
            # measure odometry from rear wheels
            name = b'bl_wheel_joint'
            name2 = b'br_wheel_joint'
            left = WHEEL_RADIUS * diff[self.joint_name.index(name)]
            right = WHEEL_RADIUS * diff[self.joint_name.index(name2)]
            dist = (left + right)/2
            angle = (right - left)/WHEEL_SEPARATION_WIDTH
            x += math.cos(heading) * dist
            y += math.sin(heading) * dist
            heading += angle
        self.prev_position = data[:]
        self.pose2d = x, y, heading
        return self.pose2d


# vim: expandtab sw=4 ts=4
