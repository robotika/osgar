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
        self.circle_limit = math.radians(10)
        self.debug_arr = []

    def is_crab_step(self, steering):
        avg = sum(steering)/4
        offset = max([abs(s - avg) for s in steering])
        return offset < self.crab_limit

    def is_turn_in_place(self, steering):
        fl, fr, bl, br = steering
        lim = self.turn_in_place_limit
        return fl < -lim and fr > lim and bl > lim and br < -lim

    def is_on_circle(self, steering):
        fl, fr, bl, br = steering
        return abs(fl + bl) < self.circle_limit and abs(fr + br) < self.circle_limit

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
        drive = [WHEEL_RADIUS * diff[names.index(n + b'_wheel_joint')]
                 for n in WHEEL_NAMES]
        if self.is_crab_step(steering):
            dist = sum(drive)/4
            # TODO use rather abs min distance with sign
            angle = sum(steering)/4  # all wheels point the same direction
            x += math.cos(heading + angle) * dist
            y += math.sin(heading + angle) * dist
            # expected no change of heading
        elif self.is_turn_in_place(steering):
            # not expected change in x and y coordinates
            # average distance driven on wheel on the circle
            dist = (-drive[0] + drive[1] - drive[2] + drive[3])/4
            radius = math.hypot(WHEEL_SEPARATION_WIDTH/2, WHEEL_SEPARATION_LENGTH/2)
            heading += dist/radius
        elif self.is_on_circle(steering):
            # the angular speed is identical on the whole robot body
            # angular_speed = wheel_speed/driving_wheel_radius
            # sin(steering_angle) = (L/2)/driving_wheel_radius
            left_angular = ((drive[0] * math.sin(steering[0]) - drive[1] * math.sin(steering[1]))/(WHEEL_SEPARATION_LENGTH/2))/2
            right_angular = ((drive[2] * math.sin(steering[2]) - drive[3] * math.sin(steering[3]))/(WHEEL_SEPARATION_LENGTH/2))/2
            angle = (left_angular + right_angular)/2
            dist = sum(drive)/4  # TODO correct it
            x += math.cos(heading) * dist
            y += math.sin(heading) * dist
            heading += angle
        else:
            #assert False, (steering, drive)
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
