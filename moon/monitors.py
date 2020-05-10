"""
  Moon Monitors for easier strategy coding
"""

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

    def update(self, robot, channel):
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


# vim: expandtab sw=4 ts=4
