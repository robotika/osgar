"""
  Moon Monitors for easier strategy coding
"""
import math

from osgar.lib.mathex import normalizeAnglePIPI


def min_dist(laser_data):
    if len(laser_data) > 0:
        # remove ultra near reflections and unlimited values == 0
        laser_data = [x if x > 10 else 10000 for x in laser_data]
        return min(laser_data)/1000.0
    return 0


class LidarCollisionException(Exception):
    pass


class LidarCollisionMonitor:
    def __init__(self, robot):
        self.robot = robot

    def update(self, robot, channel, data):
        if channel == 'scan':
            size = len(robot.scan)
            # measure distance in front of the rover = 180deg of 270deg
            if min_dist(robot.scan[size//6:-size//6]) < 1.0:
                raise LidarCollisionException()

    # context manager functions
    def __enter__(self):
        self.callback = self.robot.register(self.update)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.robot.unregister(self.callback)


class VirtualBumperException(Exception):
    pass


class VirtualBumperMonitor:
    def __init__(self, robot, virtual_bumper):
        self.robot = robot
        self.virtual_bumper = virtual_bumper

    def update(self, robot, channel, data):
        if channel == 'desired_speed':
            assert len(data) == 2, data
            speed, angular_speed = data[0]/1000.0, math.radians(data[1]/100.0)
            self.virtual_bumper.update_desired_speed(speed, angular_speed)
        elif channel == 'pose2d':
            x, y, heading = data
            pose = (x / 1000.0, y / 1000.0, math.radians(heading / 100.0))
            self.virtual_bumper.update_pose(robot.time, pose)  # hmm, the time here is not nice
            if self.virtual_bumper.collision():
                raise VirtualBumperException()

    # context manager functions
    def __enter__(self):
        self.callback = self.robot.register(self.update)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.robot.unregister(self.callback)


class PitchRollException(Exception):
    pass


class PitchRollMonitor:
    def __init__(self, robot):
        self.robot = robot

    def update(self, robot, channel, data):
        # TODO use orientation with quaternion instead
        if channel == 'rot':
            yaw, pitch, roll = [normalizeAnglePIPI(math.radians(x/100)) for x in data]
            if pitch > 0.5:
                raise PitchRollException()

    # context manager functions
    def __enter__(self):
        self.callback = self.robot.register(self.update)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.robot.unregister(self.callback)

# vim: expandtab sw=4 ts=4
