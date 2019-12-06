"""
  Explore - follow a wall/obstacles
    (use 2D SICK LIDAR only)
"""
import math
from datetime import timedelta

import numpy as np

from osgar.node import Node


WALL_DISTANCE = 1.5  #0.7  # m
DESIRED_SPEED = 0.5  # m/s

BLIND_ZONE_MM = 100  # self-reflections of the laser itself


def min_dist(data):
    data = np.array(data)
    mask = (data > BLIND_ZONE_MM)  # reject ultra close reflections and 0 = no reflection
    if np.any(mask):
        return np.min(data[mask]) * 0.001
    return None 


def tangent_circle(dist, radius):
    """
      return tangent angle to a circle placed at (dist, 0.0) with radius=radius

      For non-existing tangent use 100 degrees
          and too far (2 times radius) circle return None
    """
    if dist < 3 * radius:
        if dist >= radius:
            return math.asin(radius/float(dist))
        return math.radians(100)
    return None


def follow_wall_angle(laser_data, radius, right_wall=False):
    """
        Find the angle to the closest point in laser scan (either on the left or right side).
        Then calculate an angle to a free space as tangent to circle of given radius.
        This angle is returned and can be used for steering command.
    """
    data = np.array(laser_data)
    size = len(laser_data)
    deg_resolution = 270 / (size - 1)  # SICK uses extra the first and the last, i.e. 271 rays for 1 degree resolution
    mask = (data <= 300)  # ignore internal reflections
    data[mask] = 20000
    if right_wall:
        index = np.argmin(data[:size//2])  # only right side
    else:
        index = size//2 + np.argmin(data[size//2:])  # only left side
    dist = data[index]/1000.0
    laser_angle = math.radians(-135 + index * deg_resolution)
    angle = tangent_circle(dist, radius)
    if angle is not None:
        # print '(%d, %.3f) %.1f' % (index, dist, math.degrees(laser_angle + angle))
        if right_wall:
            return laser_angle + angle
        else:
            return laser_angle - angle
    # If the desired wall is out of range, we need to slowly turn to the correct side.
    return math.radians(-20 if right_wall else 20)


class FollowWall(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('desired_speed')
        self.right_wall = config.get('right_wall', False)
        self.max_speed = DESIRED_SPEED

    def send_speed_cmd(self, speed, angular_speed):
        return self.publish('desired_speed', [round(speed*1000), round(math.degrees(angular_speed)*100)])

    def update(self):  # hack, this method should be called run instead!       
        channel = super().update()  # define self.time

        desired_speed = 0.0
        start_time = self.time
        while self.time - start_time < timedelta(minutes=1):
            channel = super().update()
            if channel == 'scan':
                size = len(self.scan)
                dist = min_dist(self.scan[size//3:2*size//3])
                tangent_angle = follow_wall_angle(self.scan, right_wall=self.right_wall,
                                                  radius=WALL_DISTANCE)
                if dist < 2.0:
                    desired_speed = (self.max_speed / 2) * (dist - 0.4) / 1.6
                else:
                    desired_speed = self.max_speed
                desired_angular_speed = 0.7 * tangent_angle  # TODO better P-regulator based on angle and free space
                self.send_speed_cmd(desired_speed, desired_angular_speed)

            elif channel == 'emergency_stop':
                self.send_speed_cmd(0.0, 0.0)
                self.request_stop()  # it should be "delayed"


if __name__ == "__main__":
    from osgar.launcher import launch

    launch(app=FollowWall, description='Follow Wall', prefix='wall-')

# vim: expandtab sw=4 ts=4 

