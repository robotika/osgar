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

      For non-existing tangent use 100 degrees.
    """
    if dist >= radius:
        return math.asin(radius/float(dist))
    return math.radians(100)


def follow_wall_angle(laser_data, radius, right_wall=False, max_obstacle_distance=4.0, debug_callback=None):
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

    mask = (data >= max_obstacle_distance * 1000)  # ignore obstacles beyond 4m
    data[mask] = 20000

    # To make the code simpler, let's pretend we follow the right wall and flip
    # the result in the end if we are actually following the left wall.
    if not right_wall:
        data = data[::-1]

    distances = data / 1000.0

    r = radius * 1  # TODO: Or some other first guess?
    INCREMENT = 0.3  # In meters. TODO: Some other value, maybe?
    found_wall = False
    for attempt in range(50):
        # Find where the wall we follow starts.
        wall_start_idx = -1
        # We only accept walls to the right of the robot.
        for (i, distance) in enumerate(distances[:size//2]):
            if distance <= r:
                wall_start_idx = i
                break
        if wall_start_idx < 0:
            r += INCREMENT
        else:
            found_wall = True
            break

    if not found_wall:
        # No wall found. Let's slowly circle.
        # TODO: ideally, this would be stateful and we would spiral.
        return math.radians(-20 if right_wall else 20)

    last_wall_idx = wall_start_idx
    while True:
        last_wall_distance = distances[last_wall_idx]
        found_countinuation = False
        for i in range(last_wall_idx + 1, size):
            dist = distances[i]
            rel_idx = i - last_wall_idx
            sin_angle = math.sin(rel_idx * math.radians(deg_resolution))
            cos_angle = math.cos(rel_idx * math.radians(deg_resolution))
            # How far is the currently observed point from the previous wall point?
            gap = math.hypot(cos_angle * dist - last_wall_distance,
                             sin_angle * dist - 0)
            if gap <= radius:
                last_wall_idx = i
                found_countinuation = True
                break
        if not found_countinuation:
            break

    # If we do not see the end of the wall because of occlusion, our desired
    # direction is just towards the last wall point we see. Only otherwise we
    # can keep a safe distance from the wall we follow.
    last_wall_distance = distances[last_wall_idx]
    if last_wall_idx + 1 < size and distances[last_wall_idx + 1] < last_wall_distance:
        tangent_angle = 0.
    else:
        tangent_angle = tangent_circle(last_wall_distance, radius)

    laser_angle = math.radians(-135 + last_wall_idx * deg_resolution)
    total_angle = laser_angle + tangent_angle
    if not right_wall:
        total_angle = -total_angle

    if debug_callback is not None:

        def deg(index):
            ret = -135 + index * deg_resolution
            return ret if right_wall else -ret

        def rad(index):
            return math.radians(deg(index))

        debug_callback(laser_data, max_obstacle_distance, rad(wall_start_idx), rad(last_wall_idx), total_angle)

    return total_angle


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

