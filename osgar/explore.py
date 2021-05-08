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


def follow_wall_angle(laser_data, gap_size, wall_dist, right_wall=False, internal_reflection_threshold=0.3, max_wall_distance=4):
    """
        Find the angle to the closest point in laser scan (either on the left or right side).
        Then calculate an angle to a free space as tangent to circle of given wall_dist.
        This angle is returned and can be used for steering command.
    """
    data = np.array(laser_data)
    size = len(laser_data)
    deg_resolution = 270 / (size - 1)  # SICK uses extra the first and the last, i.e. 271 rays for 1 degree resolution
    rad_resolution = math.radians(deg_resolution)
    internal_reflection_threshold *= 1000 # m -> mm
    mask = (data <= internal_reflection_threshold)  # ignore internal reflections
    data[mask] = 0

    # To make the code simpler, let's pretend we follow the right wall and flip
    # the result in the end if we are actually following the left wall.
    if not right_wall:
        data = data[::-1]

    distances = data / 1000.0

    r = wall_dist * 1.2  # TODO: Or some other first guess?
    INCREMENT = 0.3  # In meters. TODO: Some other value, maybe?
    found_wall = False
    for attempt in range(50):
        # Find where the wall we follow starts.
        wall_start_idx = -1
        # We only accept walls to the right of the robot.
        for (i, distance) in enumerate(distances[:size//2]):
            if distance > max_wall_distance or distance == 0:
                continue
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
    gap_end_idx = None
    gap_end_dist = None
    while True:
        last_wall_distance = distances[last_wall_idx]
        found_countinuation = False
        for i in range(last_wall_idx + 1, size):
            dist = distances[i]
            if dist > max_wall_distance or dist == 0:
                continue
            rel_idx = i - last_wall_idx
            if rel_idx * deg_resolution >= 180:
                # We are connecting two points with a line behind the robot.
                # Such a wall does not matter for navigation and should not
                # block the robot.
                break
            sin_rel_angle = math.sin(rel_idx * rad_resolution)
            cos_rel_angle = math.cos(rel_idx * rad_resolution)
            # How far is the currently observed point from the previous wall point?
            gap = math.hypot(cos_rel_angle * dist - last_wall_distance,
                             sin_rel_angle * dist - 0)
            if gap <= gap_size:
                last_wall_idx = i
                found_countinuation = True
                gap_end_idx = None
                gap_end_dist = None
                break

            # If the gap continues already behind the robot and the continuation
            # goes roughly the in the current direction of the robot, it is likely
            # still the wall we are following.
            #
            # There is a risk here that we miss door in the wall we are following.
            # This should, however, not happen, because doors have their non-zero
            # width doorframes that we should detect as a wall perpendicular to
            # robot's direction.
            if i * deg_resolution <= 90:
                next_one = None
                for j in range(i + 1, size):
                    if distances[j] > max_wall_distance or distances[j] == 0:
                        continue
                    ridx = j - i
                    if ridx * deg_resolution >= 180:
                        break
                    sra = math.sin(ridx * rad_resolution)
                    cra = math.cos(ridx * rad_resolution)
                    g = math.hypot(cra * distances[j] - dist,
                                   sra * distances[j]  - 0)
                    if g <= gap_size:
                        next_one = j
                        break
                if next_one is not None:
                    angle = math.radians(-135 + i * deg_resolution)
                    next_dist = distances[i+1]
                    next_angle = math.radians(-135 + next_one * deg_resolution)
                    wall_direction = math.atan2(
                            next_dist * math.sin(next_angle) - dist * math.sin(angle),
                            next_dist * math.cos(next_angle) - dist * math.cos(angle))
                    if abs(wall_direction) < math.radians(25):
                        last_wall_idx = i
                        found_countinuation = True
                        gap_end_idx = None
                        gap_end_dist = None
                        break


            if gap_end_idx is None or gap < gap_end_dist:
                gap_end_idx = i
                gap_end_dist = gap
        if not found_countinuation:
            break

    # If we do not see the end of the wall because of occlusion, our desired
    # direction is just towards the last wall point we see. Only otherwise we
    # can keep a safe distance from the wall we follow.
    last_wall_distance = distances[last_wall_idx]
    # We can ignore gap ends that are already past the robot.
    if gap_end_idx is not None and -135 + gap_end_idx * deg_resolution < 90 and (gap_end_idx - last_wall_idx) * deg_resolution < 180:
        rel_idx = gap_end_idx - last_wall_idx
        if gap_end_dist <= 2 * wall_dist:
            # If the gap is too narrow, we aim into the middle of it.
            extra_angle = rel_idx * rad_resolution / 2
        else:
            # Otherwise we aim far from the wall in the direction of the gap end.
            sin_angle = math.sin(rel_idx * rad_resolution)
            cos_angle = math.cos(rel_idx * rad_resolution)
            gap_start_x = last_wall_distance
            gap_start_y = 0
            gap_end_x = cos_angle * gap_end_dist
            gap_end_y = sin_angle * gap_end_dist
            r = wall_dist / gap_end_dist
            target_x = gap_start_x + r * (gap_end_x - gap_start_x)
            target_y = gap_start_y + r * (gap_end_y - gap_start_y)
            extra_angle = math.atan2(target_y, target_x)
    else:
        extra_angle = tangent_circle(last_wall_distance, wall_dist)


    laser_angle = math.radians(-135 + last_wall_idx * deg_resolution)
    total_angle = laser_angle + extra_angle
    if not right_wall:
        total_angle = -total_angle

    return total_angle


class FollowWall(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('desired_speed')
        self.right_wall = config.get('right_wall', False)
        self.params = config.get('params', {})
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
                                                  radius=WALL_DISTANCE, **self.params)
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

