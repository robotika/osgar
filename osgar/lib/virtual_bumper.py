"""
  Virtual Bumper
  - origin from 2002 Cleaning Contest
  - if robot is not moving and receive commands to move then it is stuck probably
"""
import math


def equal_poses(pose1, pose2, dist_limit):
    x1, y1, heading1 = pose1
    x2, y2, heading2 = pose2
    return math.hypot(x2 - x1, y2 - y1) < dist_limit


class VirtualBumper:
    def __init__(self, stuck_time_limit, dist_radius_limit):
        self.stuck_time_limit = stuck_time_limit
        self.dist_radius_limit = dist_radius_limit
        self.should_be_moving = False  # status based on desired speed commands
        self.verbose = False
        self.reset_counters()

    def reset_counters(self):
        self.last_pose = None
        self.last_pose_time = None  # time of last pose without motion
        self.not_moving = None  # how long it is not moving?

    def update_desired_speed(self, desired_speed, desired_angular_speed):
        self.should_be_moving = abs(desired_speed) > 0.001 or abs(desired_angular_speed) > 0.001
        if self.verbose:
            print('VirtualBumper', desired_speed, desired_angular_speed, self.should_be_moving)
        if not self.should_be_moving:
            self.reset_counters()

    def update_pose(self, timestamp, pose):
        if self.verbose:
            print('VirtualBumper', timestamp, pose)
        if self.last_pose is not None:
            if equal_poses(pose, self.last_pose, self.dist_radius_limit):
                self.not_moving = timestamp - self.last_pose_time
            else:
                self.last_pose = pose
                self.last_pose_time = timestamp
        else:
            self.last_pose = pose
            self.last_pose_time = timestamp
        if self.verbose:
            print(self.last_pose, self.last_pose_time, self.not_moving, self.stuck_time)
        # if it should not be moving reset all counters
        if not self.should_be_moving:
            self.reset_counters()

    def collision(self):
        if self.not_moving is None:
            return False
        return self.not_moving >= self.stuck_time_limit

# vim: expandtab sw=4 ts=4
