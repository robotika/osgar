"""
  Virtual Bumper
  - origin from 2002 Cleaning Contest
  - if robot is not moving and receive commands to move then it is stuck probably
"""
import math


def equal_poses(pose1, pose2):
    x1, y1, heading1 = pose1
    x2, y2, heading2 = pose2
    return abs(x1 - x2) < 0.01 and abs(y1 - y2) < 0.01 and abs(heading1 - heading2) < math.radians(1)


class VirtualBumper:
    def __init__(self, stuck_time):
        self.stuck_time = stuck_time
        self.should_be_moving = False
        self.last_pose = None
        self.last_pose_time = None
        self.not_moving = None

    def update_desired_speed(self, desired_speed, desired_angular_speed):
        self.should_be_moving = abs(desired_speed) > 0.001 or abs(desired_angular_speed) > 0.001
        if not self.should_be_moving:
            self.last_pose = None
            self.last_pose_time = None
            self.not_moving = None

    def update_pose(self, timestamp, pose):
        if self.last_pose is not None:
            if equal_poses(pose, self.last_pose):
                self.not_moving = timestamp - self.last_pose_time
            else:
                self.last_pose = pose
                self.last_pose_time = timestamp
        else:
            self.last_pose = pose
            self.last_pose_time = timestamp

    def collision(self):
        if self.not_moving is None:
            return False
        return self.not_moving >= self.stuck_time

# vim: expandtab sw=4 ts=4
