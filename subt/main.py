"""
  SubT Challenge Version 1
"""
import gc
import os.path
import math
import threading
import copy

from datetime import timedelta
from collections import defaultdict
from io import StringIO

import numpy as np

from osgar.explore import follow_wall_angle
from osgar.lib.mathex import normalizeAnglePIPI
from osgar.lib import quaternion
from osgar.lib.virtual_bumper import VirtualBumper
from osgar.lib.lidar_pts import equal_scans
from osgar.lib.loop import LoopDetector

from subt.local_planner import LocalPlanner
from subt.trace import Trace, distance3D
from subt.name_decoder import parse_robot_name

# safety limits for exploration and return home
DEFAULT_LIMIT_ROLL = 28  # degrees
DEFAULT_LIMIT_PITCH = 28  # degrees
DEFAULT_RETURN_LIMIT_ROLL = 35  # degrees
DEFAULT_RETURN_LIMIT_PITCH = 35  # degrees

# accepted LoRa commands
LORA_GO_HOME_CMD = b'GoHome'
LORA_STOP_CMD = b'Stop'
LORA_PAUSE_CMD = b'Pause'
LORA_CONTINUE_CMD = b'Continue'

# reasons for termination of follow wall
REASON_DIST_REACHED = 'dist_reached'
REASON_PITCH_LIMIT = 'pitch_limit'
REASON_ROLL_LIMIT = 'roll_limit'
REASON_VIRTUAL_BUMPER = 'virtual_bumper'
REASON_LORA = 'lora'
REASON_FRONT_BUMPER = 'front_bumper'
REASON_REAR_BUMPER = 'rear_bumper'
REASON_LOOP = 'loop'


def any_is_none(*args):
    for a in args:
        if a is None:
            return True
    return False


def min_dist(laser_data):
    if len(laser_data) > 0:
        # remove ultra near reflections and unlimited values == 0
        laser_data = [x if x > 10 else 10000 for x in laser_data]
        return min(laser_data)/1000.0
    return 0


def distance(pose1, pose2):
    return math.hypot(pose1[0] - pose2[0], pose1[1] - pose2[1])


class Collision(Exception):
    pass


class EmergencyStopException(Exception):
    pass


class NewWaypointsException(Exception):
    pass


class EmergencyStopMonitor:
    def __init__(self, robot):
        self.robot = robot

    def update(self, robot):
        if robot.emergency_stop:
            raise EmergencyStopException()

        # handle STOP independently of current subroutine
        if robot.lora_cmd == LORA_STOP_CMD:
            print(robot.time, 'LoRa cmd - Stop')
            raise EmergencyStopException()

    # context manager functions
    def __enter__(self):
        self.callback = self.robot.register(self.update)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.robot.unregister(self.callback)


class NewWaypointsMonitor:
    def __init__(self, robot):
        self.robot = robot

    def update(self, robot):
        if robot.waypoints is not None:
            raise NewWaypointsException()

    # context manager functions
    def __enter__(self):
        self.callback = self.robot.register(self.update)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.robot.unregister(self.callback)


class SubTChallenge:
    def __init__(self, config, bus):
        self.bus = bus
        bus.register("desired_speed", "pose2d", "stdout", "desired_z_speed", "flipped")
        self.traveled_dist = 0.0
        self.time = None
        self.max_speed = config['max_speed']
        self.max_angular_speed = math.radians(60)
        self.rotation_p = config.get('rotation_p', 0.8)
        self.turbo_speed = config.get('turbo_speed')
        self.cautious_speed = config.get('cautious_speed')
        self.min_speed = config.get('min_speed', 0.05)
        self.gap_size = config['gap_size']
        self.wall_dist = config['wall_dist']
        self.follow_wall_params = config.get('follow_wall', {})
        self.timeout = timedelta(seconds=config['timeout'])
        self.symmetric = config['symmetric']  # is robot symmetric?
        self.dangerous_dist = config.get('dangerous_dist', 0.3)
        self.min_safe_dist = config.get('min_safe_dist', 0.75)
        self.turbo_safe_dist = config.get('turbo_safe_dist')
        self.safety_turning_coeff = config.get('safety_turning_coeff', 0.8)
        self.limit_pitch = math.radians(config.get('limit_pitch', DEFAULT_LIMIT_PITCH))
        self.limit_roll = math.radians(config.get('limit_roll', DEFAULT_LIMIT_ROLL))
        self.return_limit_pitch = math.radians(config.get('return_limit_pitch', DEFAULT_RETURN_LIMIT_PITCH))
        self.return_limit_roll = math.radians(config.get('return_limit_roll', DEFAULT_RETURN_LIMIT_ROLL))
        virtual_bumper_sec = config.get('virtual_bumper_sec')
        self.virtual_bumper = None
        if virtual_bumper_sec is not None:
            virtual_bumper_radius = config.get('virtual_bumper_radius', 0.1)
            self.virtual_bumper = VirtualBumper(timedelta(seconds=virtual_bumper_sec), virtual_bumper_radius)
        self.speed_policy = config.get('speed_policy', 'always_forward')
        assert(self.speed_policy in ['always_forward', 'conservative'])
        self.height_above_ground = config.get('height_above_ground', 0.0)
        self.trace_z_weight = config.get('trace_z_weight', 0.2)  # Z is important for drones ~ 3.0
        self.neighborhood_size = config.get('neighborhood_size', 12.0)
        self.approach_angle = math.radians(config.get('approach_angle', 45))

        self.last_position = (0, 0, 0)  # proper should be None, but we really start from zero
        self.xyz = None  # unknown initial 3D position
        self.yaw, self.pitch, self.roll = 0, 0, 0
        self.orientation = None  # quaternion updated by on_pose3d()
        self.yaw_offset = None  # not defined, use first IMU reading
        self.is_moving = None  # unknown
        self.scan = None  # I should use class Node instead
        self.slopes = None
        self.flipped = False  # by default use only front part
        self.flipping_pose = self.last_position  # Last position where the robot flipped.
        self.joint_angle_rad = []  # optinal angles, needed for articulated robots flip
        self.stat = defaultdict(int)
        self.voltage = []
        self.whereabouts = {}  # Whereabouts of other robots.
        self.trace = Trace()
        self.waypoints = None  # external waypoints for navigation
        self.loop_detector = LoopDetector()
        self.collision_detector_enabled = False
        self.sim_time_sec = 0

        self.lora_cmd = None

        self.emergency_stop = None
        self.monitors = []  # for Emergency Stop Exception

        self.use_right_wall = config['right_wall']
        self.use_center = False  # navigate into center area (controlled by name ending by 'C')
        self.is_virtual = config.get('virtual_world', False)  # workaround to handle tunnel differences

        self.front_bumper = False
        self.rear_bumper = False

        self.last_send_time = None

        self.init_path = None
        if 'init_path' in config:
            pts_s = [s.split(',') for s in config['init_path'].split(';')]
            self.init_path = [(float(x), float(y)) for x, y in pts_s]
        self.robot_name = None
        scan_subsample = config.get('scan_subsample', 1)
        obstacle_influence = config.get('obstacle_influence', 0.8)
        direction_adherence = math.radians(config.get('direction_adherence', 90))
        max_obstacle_distance = config.get('max_obstacle_distance', 2.5)
        self.local_planner = LocalPlanner(
                obstacle_influence=obstacle_influence,
                direction_adherence=direction_adherence,
                max_obstacle_distance=max_obstacle_distance,
                scan_subsample=scan_subsample,
                max_considered_obstacles=100)
        self.use_return_trace = config.get('use_return_trace', True)
        self.ref_scan = None
        self.pause_start_time = None
        if config.get('start_paused', False):
            self.pause_start_time = timedelta()  # paused from the very beginning

    def is_home(self):
        HOME_RADIUS = 20.0
        home_position = self.trace.start_position()
        return home_position is None or distance3D(self.last_position, home_position) < HOME_RADIUS

    def is_approaching_another_robot(self):
        if self.neighborhood_size is None or self.approach_angle is None:
            return False

        for other_xyz in self.whereabouts.values():
            d = distance3D(self.xyz, other_xyz)
            angle = normalizeAnglePIPI(math.atan2(other_xyz[1] - self.xyz[1], other_xyz[0] - self.xyz[0]) - self.yaw)
            if d <= self.neighborhood_size and abs(angle) < self.approach_angle:
                return True

        return False

    def speed_limit(self):
        size = len(self.scan)
        dist = min_dist(self.scan[size//2-size//8:size//2+size//8])
        if dist < self.min_safe_dist:
            safe_speed = self.max_speed * (dist - self.dangerous_dist) / (self.min_safe_dist - self.dangerous_dist)
            desired_speed = safe_speed if self.cautious_speed is None or not self.is_approaching_another_robot() else min(safe_speed, self.cautious_speed)
            if 0 <= desired_speed < self.min_speed:
                desired_speed = self.min_speed
            elif -self.min_speed < desired_speed < 0:
                desired_speed = -self.min_speed
            return desired_speed
        elif self.cautious_speed is not None and self.is_approaching_another_robot():
            return self.cautious_speed
        elif self.turbo_safe_dist is not None and self.turbo_speed is not None and dist > self.turbo_safe_dist and not self.is_home():
            return self.turbo_speed
        return self.max_speed

    def send_speed_cmd(self, speed, angular_speed):
        if self.virtual_bumper is not None:
            self.virtual_bumper.update_desired_speed(speed, angular_speed)
        self.bus.publish('desired_speed', [round(speed*1000), round(math.degrees(angular_speed)*100)])
        # Corresponds to gc.disable() in __main__. See a comment there for more details.
        gc.collect()

    def go_straight(self, how_far, timeout=None):
        print(self.time, "go_straight %.1f (speed: %.1f)" % (how_far, self.max_speed), self.last_position, self.flipped)
        start_pose = self.last_position
        if self.flipped:
            how_far = -how_far  # flip desired direction to current robot front
        if how_far >= 0:
            self.send_speed_cmd(self.max_speed, 0.0)
        else:
            self.send_speed_cmd(-self.max_speed, 0.0)
        start_time = self.time
        while distance(start_pose, self.last_position) < abs(how_far):
            if self.slopes:
                slope_idx = 0 if how_far < 0 else len(self.slopes) // 2
                slope = self.slopes[slope_idx]
                desired_z_speed = float(self.max_speed * math.tan(slope))
                self.bus.publish('desired_z_speed', desired_z_speed)
            self.update()
            if timeout is not None and self.time - start_time > timeout:
                print("go_straight - TIMEOUT!")
                break
        self.send_speed_cmd(0.0, 0.0)

    def go_safely(self, desired_direction, allow_z_control=True):
        if self.local_planner is None:
            safety, safe_direction = 1.0, desired_direction
        else:
            safety, safe_direction = self.local_planner.recommend(desired_direction)
        if self.flipped and self.joint_angle_rad:
            safe_direction = normalizeAnglePIPI(safe_direction + sum(self.joint_angle_rad))
        #print(self.time,"safety:%f    desired:%f  safe_direction:%f"%(safety, desired_direction, safe_direction))
        desired_angular_speed = self.rotation_p * safe_direction
        desired_speed = self.speed_limit() * (1.0 - self.safety_turning_coeff * min(self.max_angular_speed, abs(desired_angular_speed)) / self.max_angular_speed)
        if allow_z_control and self.slopes:
            slope_idx = int(len(self.slopes) * (safe_direction - -math.pi) / (2 * math.pi))
            slope = self.slopes[slope_idx]
            desired_z_speed = float(desired_speed * math.tan(slope))
            self.bus.publish('desired_z_speed', desired_z_speed)
        if self.flipped:
            self.send_speed_cmd(-desired_speed, desired_angular_speed)
        else:
            self.send_speed_cmd(desired_speed, desired_angular_speed)
        return safety

    def turn(self, angle, with_stop=True, speed=0.0, timeout=None):
        print(self.time, "turn %.1f" % math.degrees(angle))

        if self.slopes:
            self.bus.publish('desired_z_speed', 0)

        start_pose = self.last_position
        if angle >= 0:
            self.send_speed_cmd(speed, self.max_angular_speed)
        else:
            self.send_speed_cmd(speed, -self.max_angular_speed)
        start_time = self.time
        while abs(normalizeAnglePIPI(start_pose[2] - self.last_position[2])) < abs(angle):
            self.update()
            if timeout is not None and self.time - start_time > timeout:
                print(self.time, "turn - TIMEOUT!")
                break
        if with_stop:
            self.stop(2)

    def stop(self, timeout_sec=20):
        self.send_speed_cmd(0.0, 0.0)

        if self.slopes:
            self.bus.publish('desired_z_speed', 0)

        start_time = self.time
        while self.time - start_time < timedelta(seconds=timeout_sec):
            self.update()
            if not self.is_moving:
                break
        print(self.time, 'stop at', self.time - start_time, self.is_moving)


    def can_flip(self):
        return self.symmetric and distance3D(self.last_position, self.flipping_pose) > 1.0

    def flip(self, with_stop=True):
        # make sure that we will use clean data
        self.scan = None

        self.flipped = not self.flipped
        self.flipping_pose = self.last_position
        self.bus.publish('flipped', self.flipped)

        if with_stop:
            self.stop(2)

        # Making sure we have scan data. Parts of the code assume that.
        while self.scan is None:
            self.update()


    def follow_wall(self, gap_size, wall_dist, right_wall=False, timeout=timedelta(hours=3), dist_limit=None,
            pitch_limit=None, roll_limit=None, detect_loop=True):
        reason = None  # termination reason is not defined yet
        start_dist = self.traveled_dist
        start_time = self.sim_time_sec

        last_pause_time = timedelta()  # for multiple Pause
        current_pause_time = timedelta()
        while self.sim_time_sec - start_time < (timeout + last_pause_time + current_pause_time).total_seconds():
            try:
                channel = self.update()
                if (channel == 'scan' and not self.flipped) or (channel == 'scan_back' and self.flipped) or channel == 'scan360':
                    if self.pause_start_time is None:
                        if self.use_center:
                            self.go_safely(0)
                        else:
                            desired_direction = normalizeAnglePIPI(
                                    follow_wall_angle(self.scan, gap_size=gap_size, wall_dist=wall_dist, right_wall=right_wall, **self.follow_wall_params))
                            flip_threshold = math.radians(130)  # including some margin around corners
                            if self.can_flip() and (
                                    (right_wall and desired_direction > flip_threshold) or
                                    (not right_wall and desired_direction < -flip_threshold)):
                                print('Flipping:', math.degrees(desired_direction))
                                self.flip()
                            else:
                                self.go_safely(desired_direction)
                if dist_limit is not None:
                    if dist_limit < abs(self.traveled_dist - start_dist):  # robot can return backward -> abs()
                        print(self.time, 'Distance limit reached! At', self.traveled_dist, self.traveled_dist - start_dist)
                        reason = REASON_DIST_REACHED
                        break
                if pitch_limit is not None and self.pitch is not None:
                    if abs(self.pitch) > pitch_limit:
                        print(self.time, 'Pitch limit triggered termination: (pitch %.1f)' % math.degrees(self.pitch))
                        reason = REASON_PITCH_LIMIT
                        break
                if roll_limit is not None and self.roll is not None:
                    if abs(self.roll) > roll_limit:
                        print(self.time, 'Roll limit triggered termination: (roll %.1f)' % math.degrees(self.roll))
                        reason = REASON_ROLL_LIMIT
                        break
                if self.virtual_bumper is not None and self.virtual_bumper.collision():
                    print(self.time, "VIRTUAL BUMPER - collision")
                    self.go_straight(-0.3, timeout=timedelta(seconds=10))
                    reason = REASON_VIRTUAL_BUMPER
                    break

                loop = detect_loop and self.loop_detector.loop()
                if loop:
                    print(self.time, self.sim_time_sec, 'Loop detected')
                    turn_timeout = timedelta(seconds=40)
                    if self.symmetric:
                        self.flip()
                        if self.speed_policy == 'conservative':
                            self.turn(math.radians(-20 if self.use_right_wall else 20), timeout=turn_timeout, with_stop=True)
                    else:
                        self.turn(math.radians(160 if self.use_right_wall else -160), timeout=turn_timeout, with_stop=True)
                    reason = REASON_LOOP
                    break

                if self.front_bumper and not self.flipped:
                    print(self.time, "FRONT BUMPER - collision")
                    self.go_straight(-0.3, timeout=timedelta(seconds=10))
                    reason = REASON_FRONT_BUMPER
                    break

                if self.rear_bumper and self.flipped:
                    print(self.time, "REAR BUMPER - collision")
                    self.go_straight(-0.3, timeout=timedelta(seconds=10))
                    reason = REASON_REAR_BUMPER
                    break

                if self.lora_cmd is not None:
                    # the "GoHome" command must be accepted only on the way there and not on the return home
                    if dist_limit is None and self.lora_cmd == LORA_GO_HOME_CMD:
                        print(self.time, 'LoRa cmd - GoHome')
                        self.lora_cmd = None
                        reason = REASON_LORA
                        break
                    if self.lora_cmd == LORA_PAUSE_CMD:
                        print(self.time, 'LoRa cmd - Pause')
                        self.send_speed_cmd(0, 0)
                        if self.pause_start_time is None:
                            # ignore repeated Pause
                            self.pause_start_time = self.time
                        self.lora_cmd = None
                    elif self.lora_cmd == LORA_CONTINUE_CMD:
                        print(self.time, 'LoRa cmd - Continue')
                        if self.pause_start_time is not None:
                            # ignore Continue without Pause
                            last_pause_time += self.time - self.pause_start_time
                            self.pause_start_time = None
                        self.lora_cmd = None
                if self.pause_start_time is not None:
                    current_pause_time = self.time - self.pause_start_time
                else:
                    current_pause_time = timedelta()
            except Collision:
                assert not self.collision_detector_enabled  # collision disables further notification
                before_stop = self.xyz
                self.stop()
                after_stop = self.xyz
                print("Pose Jump:", before_stop, after_stop)
                self.xyz = before_stop
                self.go_straight(-1)
                self.stop()
                if right_wall:
                    turn_angle = math.pi / 2
                else:
                    turn_angle = -math.pi / 2
                self.turn(turn_angle, with_stop=True)
                self.go_straight(1.5)
                self.stop()
                self.turn(-turn_angle, with_stop=True)
                self.go_straight(1.5)
                self.stop()
                self.collision_detector_enabled = True
        self.scan = None
        return self.traveled_dist - start_dist, reason

    def return_home(self, timeout, home_threshold=None):
        if home_threshold is None:
            HOME_THRESHOLD = 5.0
        else:
            HOME_THRESHOLD = home_threshold
        SHORTCUT_RADIUS = 2.3
        MAX_TARGET_DISTANCE = 5.0
        MIN_TARGET_DISTANCE = 1.0
        assert(MAX_TARGET_DISTANCE > SHORTCUT_RADIUS) # Because otherwise we could end up with a target point more distant from home than the robot.
        print('Wait and get ready for return')
        self.send_speed_cmd(0, 0)
        self.wait(dt=timedelta(seconds=3.0))
        original_trace = copy.deepcopy(self.trace)
        self.trace.prune(SHORTCUT_RADIUS)
        self.wait(dt=timedelta(seconds=2.0))
        print('done.')
        home_position = self.trace.start_position()
        if home_position is None:
            return  # empty trace, no start -> we are at home
        start_time = self.sim_time_sec
        target_distance = MAX_TARGET_DISTANCE
        count_down = 0
        while distance3D(self.xyz, home_position) > HOME_THRESHOLD and self.sim_time_sec - start_time < timeout.total_seconds():
            channel = self.update()
            if (channel == 'scan' and not self.flipped) or (channel == 'scan_back' and self.flipped) or (channel == 'scan360'):
                if target_distance == MIN_TARGET_DISTANCE:
                    target_x, target_y, target_z = original_trace.where_to(self.xyz, target_distance, self.trace_z_weight)
                else:
                    target_x, target_y, target_z = self.trace.where_to(self.xyz, target_distance, self.trace_z_weight)
#                print(self.time, self.xyz, (target_x, target_y), math.degrees(self.yaw))
                x, y = self.xyz[:2]
                yaw = (self.yaw + math.pi) if self.flipped else self.yaw
                desired_direction = normalizeAnglePIPI(math.atan2(target_y - y, target_x - x) - yaw)
                if self.flipped and self.joint_angle_rad:
                    desired_direction = normalizeAnglePIPI(desired_direction + sum(self.joint_angle_rad))
                if self.can_flip() and abs(desired_direction) > math.radians(95):  # including hysteresis
                    print('Flipping:', math.degrees(desired_direction))
                    self.flip()
                    continue

                d = distance3D(self.xyz, [target_x, target_y, target_z])
                time_to_target = d / max(0.01, abs(self.speed_limit()))
                desired_z_speed = (target_z - self.xyz[2]) / time_to_target
                self.bus.publish('desired_z_speed', desired_z_speed)

                safety = self.go_safely(desired_direction, allow_z_control=False)
                if safety < 0.2:
                    print(self.time, "Safety low!", safety, desired_direction)
                    target_distance = MIN_TARGET_DISTANCE
                    count_down = 300
                if count_down > 0:
                    count_down -= 1
                    if count_down == 0:
                        target_distance = MAX_TARGET_DISTANCE
                        print(self.time, "Recovery to original", target_distance)

        print('return_home: dist', distance3D(self.xyz, home_position), 'time(sec)', self.sim_time_sec - start_time)
        self.bus.publish('desired_z_speed', None)

    def follow_trace(self, trace, timeout, max_target_distance=5.0, end_threshold=None, safety_limit=None, is_trace3d=False):
        print('Follow trace')
        if end_threshold is None:
            END_THRESHOLD = 2.0
        else:
            END_THRESHOLD = end_threshold
        start_time = self.sim_time_sec
        print('MD', self.xyz, distance3D(self.xyz, trace.trace[0]), trace.trace)
        while distance3D(self.xyz, trace.trace[0]) > END_THRESHOLD and self.sim_time_sec - start_time < timeout.total_seconds():
            if self.update() in ['scan', 'scan360']:
                target_x, target_y, target_z = trace.where_to(self.xyz, max_target_distance, self.trace_z_weight)
                x, y, z = self.xyz
                yaw = (self.yaw + math.pi) if self.flipped else self.yaw
                desired_direction = normalizeAnglePIPI(math.atan2(target_y - y, target_x - x) - yaw)
                if self.flipped and self.joint_angle_rad:
                    desired_direction = normalizeAnglePIPI(desired_direction + sum(self.joint_angle_rad))
                if self.can_flip() and abs(desired_direction) > math.radians(95):  # including hysteresis
                    print('Flipping:', math.degrees(desired_direction))
                    self.flip()
                else:
                    safety = self.go_safely(desired_direction, allow_z_control=False)
                    if safety_limit is not None and safety < safety_limit:
                        print('Danger! Safety limit for follow trace reached!', safety, safety_limit)
                        break

                if is_trace3d:
                    d = distance3D(self.xyz, [target_x, target_y, target_z])
                    time_to_target = d / max(0.01, abs(self.speed_limit()))
                    desired_z_speed = (target_z - self.xyz[2]) / time_to_target
                    self.bus.publish('desired_z_speed', desired_z_speed)

        print('End of follow trace(sec)', self.sim_time_sec - start_time)
        if is_trace3d:
            self.bus.publish('desired_z_speed', None)

    def register(self, callback):
        self.monitors.append(callback)
        return callback

    def unregister(self, callback):
        assert callback in self.monitors
        self.monitors.remove(callback)

    def on_pose3d(self, timestamp, data):
        xyz, rot = data
        self.orientation = rot  # quaternion
        ypr = quaternion.euler_zyx(rot)

        pose = (xyz[0], xyz[1], ypr[0])
        if self.last_position is not None:
            self.is_moving = (self.last_position != pose)
            dist = math.hypot(pose[0] - self.last_position[0], pose[1] - self.last_position[1])
            direction = ((pose[0] - self.last_position[0]) * math.cos(self.last_position[2]) +
                         (pose[1] - self.last_position[1]) * math.sin(self.last_position[2]))
            if direction < 0:
                dist = -dist
        else:
            dist = 0.0
        self.last_position = pose
        self.traveled_dist += dist
        x, y, z = xyz
        self.last_send_time = self.bus.publish('pose2d', [round(x * 1000), round(y * 1000),
                                    round(math.degrees(self.yaw) * 100)])
        if self.virtual_bumper is not None:
            if self.is_virtual:
                self.virtual_bumper.update_pose(timedelta(seconds=self.sim_time_sec), pose)
            else:
                self.virtual_bumper.update_pose(self.time, pose)
        self.xyz = tuple(xyz)
        if self.yaw_offset is None:
            self.yaw_offset = -ypr[0]
        tmp_yaw, self.pitch, self.roll = ypr
        self.yaw = tmp_yaw + self.yaw_offset
        self.trace.update_trace(self.xyz)
        self.loop_detector.add(self.xyz, rot, (self.flipped,))

    def on_acc(self, timestamp, data):
        acc = [x / 1000.0 for x in data]
        gacc = np.matrix([[0., 0., 9.80]])  # Gravitational acceleration.
        cos_pitch = math.cos(self.pitch)
        sin_pitch = math.sin(self.pitch)
        # TODO: Once roll is correct, incorporate it here too.
        egacc = np.matrix([  # Expected gravitational acceleration given known pitch.
            [cos_pitch, 0., sin_pitch],
            [0., 1., 0.],
            [-sin_pitch, 0., cos_pitch]
        ]) * gacc.T
        cacc = np.asarray(acc) - egacc.T  # Corrected acceleration (without gravitational acceleration).
        magnitude = math.hypot(cacc[0, 0], cacc[0, 1])
        # used to be 12 - see https://bitbucket.org/osrf/subt/issues/166/expected-x2-acceleration
        if magnitude > 200:  #18.0:
            print(self.time, 'Collision!', acc, 'reported:', self.collision_detector_enabled)
            if self.collision_detector_enabled:
                self.collision_detector_enabled = False
                raise Collision()

    def on_joint_angle(self, timestamp, data):
        # angles for articulated robot in 1/100th of degree
        self.joint_angle_rad = [math.radians(a/100) for a in data]

    def on_bumpers_front(self, timestamp, data):
        self.front_bumper = max(data)  # array of boolean values where True means collision

    def on_bumpers_rear(self, timestamp, data):
        self.rear_bumper = max(data)  # array of boolean values where True means collision

    def on_robot_name(self, timestamp, data):
        self.robot_name = data.decode('ascii')

    def on_waypoints(self, timestamp, data):
        self.waypoints = data

    def on_slopes(self, timestamp, data):
        self.slopes = [math.radians(a/10) for a in data]

    def on_whereabouts(self, timestamp, data):
        robot_name, (_, robot_pose) = data
        if robot_name != self.robot_name:
            self.whereabouts[robot_name] = robot_pose

    def update(self):
        packet = self.bus.listen()
        if packet is not None:
#            print('SubT', packet)
            timestamp, channel, data = packet
            if self.time is None or int(self.time.seconds)//60 != int(timestamp.seconds)//60:
                self.stdout(timestamp, '(%.1f %.1f %.1f)' % self.xyz if self.xyz is not None else '(xyz=None)', sorted(self.stat.items()))
                print(timestamp, list(('%.1f' % (v/100)) for v in self.voltage))
                self.stat.clear()

            self.time = timestamp

            if not self.is_virtual:
                self.sim_time_sec = self.time.total_seconds()

            self.stat[channel] += 1
            handler = getattr(self, "on_" + channel, None)
            if handler is not None:
                handler(timestamp, data)
            elif channel == 'scan' and not self.flipped:
                if self.last_send_time is not None and self.last_send_time - self.time > timedelta(seconds=0.1):
                    print('queue delay', self.last_send_time - self.time)
                self.scan = data
                if self.ref_scan is None or not equal_scans(self.scan, self.ref_scan, 200):
                    self.ref_scan = self.scan
                    self.ref_count = 0
                else:
                    self.ref_count += 1
                    if self.ref_count > 300:
                        print('Robot is stuck!', self.ref_count)
                        if self.collision_detector_enabled:
                            self.collision_detector_enabled = False
                            raise Collision()
                        self.ref_count = 0

                if self.local_planner is not None:
                    self.local_planner.update(data)
            elif channel == 'scan_back' and self.flipped:
                self.scan = data
                if self.local_planner is not None:
                    self.local_planner.update(data)
            elif channel == 'scan360':
                # reduce original 360 degrees scan to 270 degrees oriented forward or backward
                index45deg = int(round(len(data)/8))
                if self.flipped:
                    mid = len(data)//2
                    self.scan = (data[mid:]+data[:mid])[index45deg:-index45deg]
                else:
                    self.scan = data[index45deg:-index45deg]
                if self.local_planner is not None:
                    self.local_planner.update(self.scan)
            elif channel == 'sim_time_sec':
                self.sim_time_sec = data
            elif channel == 'voltage':
                self.voltage = data
            elif channel == 'emergency_stop':
                self.emergency_stop = data

            elif channel == 'cmd':
                self.lora_cmd = data

            for m in self.monitors:
                m(self)
            return channel

    def wait(self, dt, use_sim_time=False):  # TODO refactor to some common class
        if use_sim_time:
            start_sim_time_sec = self.sim_time_sec
            while self.sim_time_sec - start_sim_time_sec < dt.total_seconds():
                self.update()
        else:
            if self.time is None:
                self.update()
            start_time = self.time
            while self.time - start_time < dt:
                self.update()

    def stdout(self, *args, **kwargs):
        output = StringIO()
        print(*args, file=output, **kwargs)
        contents = output.getvalue().strip()
        output.close()
        self.bus.publish('stdout', contents)
        print(contents)

#############################################
    def system_nav_trace(self, path=None):
        """
        Navigate along line
        """
        x0, y0, z0 = self.xyz
        trace = Trace()
        trace.add_line_to((x0, y0, z0))
        if path is not None:
            for x, y in path:
                trace.add_line_to((x - x0, y - y0, z0))
        trace.reverse()
        self.follow_trace(trace, timeout=timedelta(seconds=120), max_target_distance=2.5, safety_limit=0.2)

    def robust_follow_wall(self, gap_size, wall_dist, right_wall=False, timeout=timedelta(hours=3), dist_limit=None,
            pitch_limit=None, roll_limit=None):
        """
        Handle multiple re-tries with increasing distance from the wall if necessary
        """
        allow_virtual_flip = self.symmetric
        wall_dist = self.wall_dist
        total_dist = 0.0
        start_time = self.sim_time_sec
        overall_timeout = timeout
        while self.sim_time_sec - start_time < overall_timeout.total_seconds():
            if self.sim_time_sec - start_time > overall_timeout.total_seconds():
                print('Total Timeout Reached', overall_timeout.total_seconds())
                break
            timeout = timedelta(seconds=overall_timeout.total_seconds() - (self.sim_time_sec - start_time))
            print('Current timeout', timeout)

            dist, reason = self.follow_wall(gap_size=gap_size, wall_dist=wall_dist, right_wall=right_wall, timeout=timeout,
                                    pitch_limit=self.limit_pitch, roll_limit=self.limit_roll)
            total_dist += dist
            if reason is None or reason in [REASON_LORA,]:
                break

            wall_dist += 0.2
            if not allow_virtual_flip:
                # Eduro not supported yet
                if reason in [REASON_VIRTUAL_BUMPER,]:
                    # virtual bumper already tried to backup a bit
                    continue
                # for large slope rather return home
                break

            # re-try with bigger distance
            print(self.time, "Re-try because of", reason)
            for repeat in range(2):
                # retreat a bit
                self.flip()
                self.follow_wall(gap_size=gap_size, wall_dist=wall_dist, right_wall=not right_wall, timeout=timedelta(seconds=30), dist_limit=2.0,
                    pitch_limit=self.return_limit_pitch, roll_limit=self.return_limit_roll)
                self.flip()

                dist, reason = self.follow_wall(gap_size=gap_size, wall_dist=wall_dist, right_wall=right_wall, timeout=timedelta(seconds=40), dist_limit=4.0,
                                        pitch_limit=self.limit_pitch, roll_limit=self.limit_roll)
                total_dist += dist
                if reason is None:
                    break
                if reason in [REASON_LORA, REASON_DIST_REACHED]:
                    break
                wall_dist += 0.2
            wall_dist = self.wall_dist
            if reason in [REASON_LORA,]:
                break


    def play_system_track(self):
        print("SubT Challenge Ver1!")
        try:
            with EmergencyStopMonitor(self):
                allow_virtual_flip = self.symmetric
                if distance(self.xyz, (0, 0)) > 0.1 or self.init_path is not None:
                    self.system_nav_trace(self.init_path)

#                self.go_straight(2.5)  # go to the tunnel entrance - commented our for testing
                wall_dist = self.wall_dist
                total_dist = 0.0
                start_time = self.sim_time_sec
                while self.sim_time_sec - start_time < self.timeout.total_seconds():
                    if self.sim_time_sec - start_time > self.timeout.total_seconds():
                        print('Total Timeout Reached', self.timeout.total_seconds())
                        break
                    timeout = timedelta(seconds=self.timeout.total_seconds() - (self.sim_time_sec - start_time))
                    print('Current timeout', timeout)

                    dist, reason = self.follow_wall(gap_size=self.gap_size, wall_dist=wall_dist, right_wall=self.use_right_wall, timeout=timeout,
                                            pitch_limit=self.limit_pitch, roll_limit=self.limit_roll)
                    total_dist += dist
                    if reason is None or reason in [REASON_LORA,]:
                        break

                    wall_dist += 0.2
                    if not allow_virtual_flip:
                        # Eduro not supported yet
                        if reason in [REASON_VIRTUAL_BUMPER,]:
                            # virtual bumper already tried to backup a bit
                            continue
                        # for large slope rather return home
                        break

                    # re-try with bigger distance
                    print(self.time, "Re-try because of", reason)
                    for repeat in range(2):
                        if allow_virtual_flip:
                            self.flip()
                        self.follow_wall(gap_size=self.gap_size, wall_dist=wall_dist, right_wall=not self.use_right_wall, timeout=timedelta(seconds=30), dist_limit=2.0,
                            pitch_limit=self.return_limit_pitch, roll_limit=self.return_limit_roll)
                        if allow_virtual_flip:
                            self.flip()

                        dist, reason = self.follow_wall(gap_size=self.gap_size, wall_dist=wall_dist, right_wall=self.use_right_wall, timeout=timedelta(seconds=40), dist_limit=4.0,
                                                pitch_limit=self.limit_pitch, roll_limit=self.limit_roll)
                        total_dist += dist
                        if reason is None:
                            break
                        if reason in [REASON_LORA, REASON_DIST_REACHED]:
                            break
                        wall_dist += 0.2
                    wall_dist = self.wall_dist
                    if reason in [REASON_LORA,]:
                        break

                if self.use_return_trace and self.local_planner is not None:
                    self.stdout(self.time, "Going HOME %.3f" % dist, reason)
                    self.return_home(2 * self.timeout, home_threshold=1.0)
                    self.send_speed_cmd(0, 0)
                else:
                    print(self.time, "Going HOME", reason)
                    if not allow_virtual_flip:
                        self.turn(math.radians(90), timeout=timedelta(seconds=20))
                        self.turn(math.radians(90), timeout=timedelta(seconds=20))
                    else:
                        self.flip()
                    self.robust_follow_wall(gap_size=self.gap_size, wall_dist=self.wall_dist, right_wall=not self.use_right_wall, timeout=3*self.timeout, dist_limit=3*total_dist,
                            pitch_limit=self.return_limit_pitch, roll_limit=self.return_limit_roll)

        except EmergencyStopException:
            print(self.time, "EMERGENCY STOP - terminating")
        self.send_speed_cmd(0, 0)
        self.wait(timedelta(seconds=3))
#############################################

    def go_to_entrance(self):
        """
        Navigate to the base station tile end
        """
        trace = Trace()
        trace.update_trace(tuple(self.xyz))
        trace.add_line_to((-2.0, 0, self.height_above_ground))  # in front of the tunnel/entrance (Finals)
        if self.use_right_wall:
            entrance_offset = -0.5
        elif self.use_center:
            entrance_offset = 0
        else:
            entrance_offset = 0.5
        trace.add_line_to((0.5, entrance_offset, self.height_above_ground))  # 0.5m inside, towards the desired wall.
        trace.reverse()
        is_trace3d = self.height_above_ground > 0.0  # well, maybe there should be more obvious definition of ground/aerial vehicle
        safety_limit = None if is_trace3d else 0.2  # UGV may need some collision avoidance during leaving the starting area
                                                    # while the CoRo Pam drone generates fake artifacts near ground
        self.follow_trace(trace, timeout=timedelta(seconds=30), max_target_distance=2.5, safety_limit=safety_limit, is_trace3d=is_trace3d)

    def play_virtual_part_explore(self):
        start_time = self.sim_time_sec
        for loop in range(100):
            self.collision_detector_enabled = True
            if self.sim_time_sec - start_time > self.timeout.total_seconds():
                print('Total Timeout Reached', self.timeout.total_seconds())
                break
            timeout = timedelta(seconds=self.timeout.total_seconds() - (self.sim_time_sec - start_time))
            print('Current timeout', timeout)

            dist, reason = self.follow_wall(gap_size=self.gap_size, wall_dist=self.wall_dist, right_wall=self.use_right_wall,
                                timeout=timeout, pitch_limit=self.limit_pitch, roll_limit=None)
            self.collision_detector_enabled = False
            if reason == REASON_VIRTUAL_BUMPER:
                assert self.virtual_bumper is not None
                self.virtual_bumper.reset_counters()

                # the robot ended in cycle probably
                self.return_home(timedelta(seconds=10))

                # try something crazy if you do not have other ideas ...
                before_center = self.use_center
                self.use_center = True
                dist, reason = self.follow_wall(gap_size=self.gap_size, wall_dist=self.wall_dist, right_wall=self.use_right_wall,
                                    timeout=timedelta(seconds=60), pitch_limit=self.limit_pitch, roll_limit=None)
                self.use_center = before_center
                if reason is None or reason != REASON_PITCH_LIMIT:
                    continue
            elif reason == REASON_LOOP:
                # Smaller wall_dist to reduce the chance that we miss the opening again that we missed before,
                # which made us end up in a loop.
                dist, reason = self.follow_wall(gap_size=self.gap_size, wall_dist=self.wall_dist*0.75, right_wall=not self.use_right_wall,
                                    timeout=timedelta(seconds=5), pitch_limit=self.limit_pitch, roll_limit=None, detect_loop=False)
                if reason is None:
                    continue

            if reason is None or reason != REASON_PITCH_LIMIT:
                break
            self.stdout(self.time, "Microstep HOME %d %.3f" % (loop, dist), reason)
            self.go_straight(-0.3, timeout=timedelta(seconds=10))
            self.return_home(timedelta(seconds=10))

        self.stdout(self.time, "Explore phase finished %.3f" % dist, reason)

    def play_virtual_part_map_and_explore_frontiers(self):
        start_time = self.sim_time_sec
        while self.sim_time_sec - start_time < self.timeout.total_seconds():
            if self.waypoints is not None:
                tmp_trace = Trace()
                tmp_trace.trace = self.waypoints
                self.waypoints = None
                tmp_trace.reverse()
                try:
                    with NewWaypointsMonitor(self) as wm:
                        self.follow_trace(tmp_trace, timeout=timedelta(seconds=10),
                                          max_target_distance=1.0, end_threshold=0.5, is_trace3d=True)
                except NewWaypointsException:
                    pass
                self.send_speed_cmd(0, 0)
            else:
                self.update()

    def play_virtual_part_return(self, timeout):
        self.return_home(timeout)
        self.send_speed_cmd(0, 0)
        self.wait(timedelta(seconds=10), use_sim_time=True)
        self.stdout('Final xyz:', self.xyz)
        self.stdout('Final xyz (DARPA coord system):', self.xyz)

    def play_virtual_track(self):
        self.stdout("SubT Challenge Ver116!")
        self.stdout("Waiting for robot_name ...")
        while self.robot_name is None:
            self.update()
        self.stdout('robot_name:', self.robot_name)

        # wait for critical data
        while any_is_none(self.scan, self.xyz):
            # self.xyz is initialized by pose3d
            self.update()

        steps = parse_robot_name(self.robot_name)
        times_sec = [duration for action, duration in steps if action != 'home' and not action.startswith('enter')]
        self.stdout('Using times', times_sec)

        for action, duration, in steps:
            if action == 'wait':
                self.stdout(f'Wait for {duration} seconds')
                self.wait(timedelta(seconds=duration), use_sim_time=True)

            elif action.startswith('enter'):
                self.use_right_wall = (action == 'enter-right')
                self.use_center = (action == 'enter-center')
                self.go_to_entrance()

            elif action in ['left', 'right', 'center']:
                self.timeout = timedelta(seconds=duration)
                self.use_right_wall = (action == 'right')
                self.use_center = (action == 'center')
                self.play_virtual_part_explore()

            elif action == 'explore':
                self.timeout = timedelta(seconds=duration)
                self.play_virtual_part_map_and_explore_frontiers()

            elif action == 'home':
                self.play_virtual_part_return(timedelta(seconds=duration))
            else:
                assert False, action  # unknown action

        self.wait(timedelta(seconds=30), use_sim_time=True)

#############################################

    def play(self):
        if self.is_virtual:
            return self.play_virtual_track()
        else:
            return self.play_system_track()

    def start(self):
        self.thread = threading.Thread(target=self.play)
        self.thread.start()

    def is_alive(self):
        return self.thread.is_alive()

    def request_stop(self):
        self.bus.shutdown()

    def join(self, timeout=None):
        self.thread.join(timeout)


def main():
    import argparse
    from osgar.lib.config import config_load
    #from osgar.record import record
    from osgar.zmqrouter import record

    parser = argparse.ArgumentParser(description='SubT Challenge')
    subparsers = parser.add_subparsers(help='sub-command help', dest='command')
    subparsers.required = True
    parser_run = subparsers.add_parser('run', help='run on real HW')
    parser_run.add_argument('config', nargs='+', help='configuration file')
    parser_run.add_argument('--note', help='add description')
    parser_run.add_argument('--gap-size', help='minimum gap the robot goeas into (default: %(default)sm)', default=1.0, type=float)
    parser_run.add_argument('--wall-dist', help='distance for wall following (default: %(default)sm)', default=1.0, type=float)
    parser_run.add_argument('--side', help='which side to follow', choices=['left', 'right', 'auto'], required=True)
    parser_run.add_argument('--speed', help='maximum speed (default: from config)', type=float)
    parser_run.add_argument('--timeout', help='seconds of exploring before going home (default: %(default)s)',
                            type=int, default=10*60)
    parser_run.add_argument('--log', nargs='?', help='record log filename')
    parser_run.add_argument('--init-path', help='inital path to be followed from (0, 0). 2D coordinates are separated by ;')
    parser_run.add_argument('--start-paused', dest='start_paused', action='store_true',
                            help='start robota Paused and wait for LoRa Contine command')

    parser_replay = subparsers.add_parser('replay', help='replay from logfile')
    parser_replay.add_argument('logfile', help='recorded log file')
    parser_replay.add_argument('--force', '-F', dest='force', action='store_true', help='force replay even for failing output asserts')
    parser_replay.add_argument('--config', nargs='+', help='force alternative configuration file')
    parser_replay.add_argument('--debug', help="print debug info about I/O streams", action='store_true')
    args = parser.parse_args()

    if args.command == 'replay':
        from osgar.replay import replay
        args.module = 'app'
        app = replay(args, application=SubTChallenge)
        app.play()

    elif args.command == 'run':
        import logging
        logging.basicConfig(
            level=logging.DEBUG,
            format='%(asctime)s %(name)-16s %(levelname)-8s %(message)s',
        )
        # To reduce latency spikes as described in https://morepypy.blogspot.com/2019/01/pypy-for-low-latency-systems.html.
        # Increased latency leads to uncontrolled behavior and robot either missing turns or hitting walls.
        # Disabled garbage collection needs to be paired with gc.collect() at place(s) that are not time sensitive.
        gc.disable()

        cfg = config_load(*args.config, application=SubTChallenge)

        # apply overrides from command line
        cfg['robot']['modules']['app']['init']['gap_size'] = args.gap_size
        cfg['robot']['modules']['app']['init']['wall_dist'] = args.wall_dist
        if args.side == 'auto':
            cfg['robot']['modules']['app']['init']['right_wall'] = 'auto'
        else:
            cfg['robot']['modules']['app']['init']['right_wall'] = args.side == 'right'
        cfg['robot']['modules']['app']['init']['timeout'] = args.timeout
        if args.init_path is not None:
            cfg['robot']['modules']['app']['init']['init_path'] = args.init_path

        if args.speed is not None:
            cfg['robot']['modules']['app']['init']['max_speed'] = args.speed

        cfg['robot']['modules']['app']['init']['start_paused'] = args.start_paused

        prefix = os.path.basename(args.config[0]).split('.')[0] + '-'
        record(cfg, prefix, args.log)


if __name__ == "__main__":
    main()

# vim: expandtab sw=4 ts=4
