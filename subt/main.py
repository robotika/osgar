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

from subt.local_planner import LocalPlanner
from subt.trace import Trace, distance3D

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


class SubTChallenge:
    def __init__(self, config, bus):
        self.bus = bus
        bus.register("desired_speed", "pose2d", "artf_xyz", "stdout", "request_origin")
        self.traveled_dist = 0.0
        self.time = None
        self.max_speed = config['max_speed']
        self.max_angular_speed = math.radians(60)
        self.walldist = config['walldist']
        self.timeout = timedelta(seconds=config['timeout'])
        self.symmetric = config['symmetric']  # is robot symmetric?
        self.dangerous_dist = config.get('dangerous_dist', 0.3)
        self.min_safe_dist = config.get('min_safe_dist', 0.75)
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

        self.last_position = (0, 0, 0)  # proper should be None, but we really start from zero
        self.xyz = None  # 3D position for mapping artifacts - unknown depends on source (pose2d or pose3d)
        self.yaw, self.pitch, self.roll = 0, 0, 0
        self.yaw_offset = None  # not defined, use first IMU reading
        self.is_moving = None  # unknown
        self.scan = None  # I should use class Node instead
        self.flipped = False  # by default use only front part
        self.joint_angle_rad = []  # optinal angles, needed for articulated robots flip
        self.stat = defaultdict(int)
        self.voltage = []
        self.artifacts = []
        self.trace = Trace()
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
        self.origin = None  # unknown initial position
        self.origin_quat = quaternion.identity()

        self.offset = None  # the offset between robot frame and world frame is not known on start
        if 'init_offset' in config:
            x, y, z = [d/1000.0 for d in config['init_offset']]
            self.offset = (x, y, z)
        self.init_path = None
        if 'init_path' in config:
            pts_s = [s.split(',') for s in config['init_path'].split(';')]
            self.init_path = [(float(x), float(y)) for x, y in pts_s]
        self.origin_error = False
        self.robot_name = None  # received with origin
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

    def send_speed_cmd(self, speed, angular_speed):
        if self.virtual_bumper is not None:
            self.virtual_bumper.update_desired_speed(speed, angular_speed)
        self.bus.publish('desired_speed', [round(speed*1000), round(math.degrees(angular_speed)*100)])
        # Corresponds to gc.disable() in __main__. See a comment there for more details.
        gc.collect()

    def maybe_remember_artifact(self, artifact_data, artifact_xyz):
        for stored_data, (x, y, z) in self.artifacts:
            if distance3D((x, y, z), artifact_xyz) < 4.0:
                # in case of uncertain type, rather report both
                if stored_data == artifact_data:
                    return False
        self.artifacts.append((artifact_data, artifact_xyz))
        return True

    def go_straight(self, how_far, timeout=None):
        print(self.time, "go_straight %.1f (speed: %.1f)" % (how_far, self.max_speed), self.last_position)
        start_pose = self.last_position
        if how_far >= 0:
            self.send_speed_cmd(self.max_speed, 0.0)
        else:
            self.send_speed_cmd(-self.max_speed, 0.0)
        start_time = self.time
        while distance(start_pose, self.last_position) < abs(how_far):
            self.update()
            if timeout is not None and self.time - start_time > timeout:
                print("go_straight - TIMEOUT!")
                break
        self.send_speed_cmd(0.0, 0.0)

    def go_safely(self, desired_direction):
        if self.local_planner is None:
            safety, safe_direction = 1.0, desired_direction
        else:
            safety, safe_direction = self.local_planner.recommend(desired_direction)
        #print(self.time,"safety:%f    desired:%f  safe_direction:%f"%(safety, desired_direction, safe_direction))
        if self.speed_policy == 'conservative':
            desired_angular_speed = 1.2 * safe_direction
        elif self.speed_policy == 'always_forward':
            desired_angular_speed = 0.9 * safe_direction
        else:
            assert(False)  # Unsupported speed_policy.
        size = len(self.scan)
        dist = min_dist(self.scan[size//3:2*size//3])
        if dist < self.min_safe_dist:  # 2.0:
            if self.speed_policy == 'conservative':
                desired_speed = self.max_speed * (1.2/2.0) * (dist - 0.4) / 1.6
            elif self.speed_policy == 'always_forward':
                desired_speed = self.max_speed * (dist - self.dangerous_dist) / (self.min_safe_dist - self.dangerous_dist)
            else:
                assert(False)  # Unsupported speed policy.
        else:
            desired_speed = self.max_speed  # was 2.0
        desired_speed = desired_speed * (1.0 - self.safety_turning_coeff * min(self.max_angular_speed, abs(desired_angular_speed)) / self.max_angular_speed)
        if self.flipped:
            self.send_speed_cmd(-desired_speed, desired_angular_speed)  # ??? angular too??!
        else:
            self.send_speed_cmd(desired_speed, desired_angular_speed)
        return safety

    def turn(self, angle, with_stop=True, speed=0.0, timeout=None):
        print(self.time, "turn %.1f" % math.degrees(angle))
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
            self.send_speed_cmd(0.0, 0.0)
            start_time = self.time
            while self.time - start_time < timedelta(seconds=2):
                self.update()
                if not self.is_moving:
                    break
            print(self.time, 'stop at', self.time - start_time)

    def stop(self):
        self.send_speed_cmd(0.0, 0.0)
        start_time = self.time
        while self.time - start_time < timedelta(seconds=20):
            self.update()
            if not self.is_moving:
                break
        print(self.time, 'stop at', self.time - start_time, self.is_moving)

    def follow_wall(self, radius, right_wall=False, timeout=timedelta(hours=3), dist_limit=None, flipped=False,
            pitch_limit=None, roll_limit=None):
        # make sure that we will start with clean data
        if flipped:
            self.scan = None
            self.flipped = True

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
                            desired_direction = 0
                        else:
                            desired_direction = follow_wall_angle(self.scan, radius=radius, right_wall=right_wall)
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

                if self.front_bumper and not flipped:
                    print(self.time, "FRONT BUMPER - collision")
                    self.go_straight(-0.3, timeout=timedelta(seconds=10))
                    reason = REASON_FRONT_BUMPER
                    break

                if self.rear_bumper and flipped:
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
        self.flipped = False
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
        start_time = self.sim_time_sec
        target_distance = MAX_TARGET_DISTANCE
        count_down = 0
        while distance3D(self.xyz, (0, 0, 0)) > HOME_THRESHOLD and self.sim_time_sec - start_time < timeout.total_seconds():
            channel = self.update()
            if (channel == 'scan' and not self.flipped) or (channel == 'scan_back' and self.flipped) or (channel == 'scan360'):
                if target_distance == MIN_TARGET_DISTANCE:
                    target_x, target_y = original_trace.where_to(self.xyz, target_distance)[:2]
                else:
                    target_x, target_y = self.trace.where_to(self.xyz, target_distance)[:2]
#                print(self.time, self.xyz, (target_x, target_y), math.degrees(self.yaw))
                x, y = self.xyz[:2]
                desired_direction = math.atan2(target_y - y, target_x - x) - self.yaw
                if self.flipped:
                    desired_direction += math.pi  # symmetry
                    for angle in self.joint_angle_rad:
                        desired_direction -= angle

                safety = self.go_safely(desired_direction)
                if safety < 0.2:
                    print(self.time, "Safety low!", safety, desired_direction)
                    target_distance = MIN_TARGET_DISTANCE
                    count_down = 300
                if count_down > 0:
                    count_down -= 1
                    if count_down == 0:
                        target_distance = MAX_TARGET_DISTANCE
                        print(self.time, "Recovery to original", target_distance)

        print('return_home: dist', distance3D(self.xyz, (0, 0, 0)), 'time(sec)', self.sim_time_sec - start_time)

    def follow_trace(self, trace, timeout, max_target_distance=5.0, safety_limit=None):
        print('Follow trace')
        END_THRESHOLD = 2.0
        start_time = self.sim_time_sec
        print('MD', self.xyz, distance3D(self.xyz, trace.trace[0]), trace.trace)
        while distance3D(self.xyz, trace.trace[0]) > END_THRESHOLD and self.sim_time_sec - start_time < timeout.total_seconds():
            if self.update() in ['scan', 'scan360']:
                target_x, target_y = trace.where_to(self.xyz, max_target_distance)[:2]
                x, y = self.xyz[:2]
                desired_direction = math.atan2(target_y - y, target_x - x) - self.yaw
                safety = self.go_safely(desired_direction)
                if safety_limit is not None and safety < safety_limit:
                    print('Danger! Safety limit for follow trace reached!', safety, safety_limit)
                    break
        print('End of follow trace(sec)', self.sim_time_sec - start_time)

    def register(self, callback):
        self.monitors.append(callback)
        return callback

    def unregister(self, callback):
        assert callback in self.monitors
        self.monitors.remove(callback)

    def on_pose2d(self, timestamp, data):
        if self.xyz is None:
            self.xyz = 0, 0, 0
        if self.offset is None:
            return
        x, y, heading = data
        pose = (x / 1000.0, y / 1000.0, math.radians(heading / 100.0))
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
        x, y, z = self.xyz
        x += math.cos(self.pitch) * math.cos(self.yaw) * dist
        y += math.cos(self.pitch) * math.sin(self.yaw) * dist
        z += math.sin(self.pitch) * dist
        x0, y0, z0 = self.offset
        self.last_send_time = self.bus.publish('pose2d', [round((x + x0) * 1000), round((y + y0) * 1000),
                                    round(math.degrees(self.yaw) * 100)])
        if self.virtual_bumper is not None:
            if self.is_virtual:
                self.virtual_bumper.update_pose(timedelta(seconds=self.sim_time_sec), pose)
            else:
                self.virtual_bumper.update_pose(self.time, pose)
        self.xyz = x, y, z
        self.trace.update_trace(self.xyz)

    def on_pose3d(self, timestamp, data):
        if self.xyz is None:
            # to avoid deadlock when we are waiting for offset but it is defined after self.xyz is not None
            self.xyz = data[0]
        if self.offset is None:
            # we cannot align global coordinates if offset is not known
            return
        xyz, rot = data
        ypr = quaternion.euler_zyx(rot)
        x0, y0, z0 = self.offset

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
        self.last_send_time = self.bus.publish('pose2d', [round((x + x0) * 1000), round((y + y0) * 1000),
                                    round(math.degrees(self.yaw) * 100)])
        if self.virtual_bumper is not None:
            if self.is_virtual:
                self.virtual_bumper.update_pose(timedelta(seconds=self.sim_time_sec), pose)
            else:
                self.virtual_bumper.update_pose(self.time, pose)
        self.xyz = tuple(xyz)
        self.trace.update_trace(self.xyz)

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

    def on_artf(self, timestamp, data):
        if self.offset is None:
            # there can be observed artifact (false) on the start before the coordinate system is defined
            return
        artifact_data, deg_100th, dist_mm = data
        x, y, z = self.xyz
        x0, y0, z0 = self.offset
        angle, dist = self.yaw + math.radians(deg_100th / 100.0), dist_mm / 1000.0
        ax = x0 + x + math.cos(angle) * dist
        ay = y0 + y + math.sin(angle) * dist
        az = z0 + z
        if -20 < ax < 0 and -10 < ay < 10:
            # filter out elements on staging area
            self.stdout(self.time, 'Robot at:', (ax, ay, az))
        else:
            if self.maybe_remember_artifact(artifact_data, (ax, ay, az)):
                self.bus.publish('artf_xyz', [[artifact_data, round(ax*1000), round(ay*1000), round(az*1000)]])

    def on_joint_angle(self, timestamp, data):
        # angles for articulated robot in 1/100th of degree
        self.joint_angle_rad = [math.radians(a/100) for a in data]

    def on_bumpers_front(self, timestamp, data):
        self.front_bumper = max(data)  # array of boolean values where True means collision

    def on_bumpers_rear(self, timestamp, data):
        self.rear_bumper = max(data)  # array of boolean values where True means collision

    def on_origin(self, timestamp, data):
        if self.origin is None:  # accept only initial offset
            self.robot_name = data[0].decode('ascii')
            if len(data) == 8:
                self.xyz_quat = data[1:4]
                self.origin = data[1:4]
                qx, qy, qz, qw = data[4:]
                self.origin_quat = qx, qy, qz, qw  # quaternion
            else:
                self.stdout('Origin ERROR received')
                self.origin_error = True

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
                    self.local_planner.update(data)
            elif channel == 'rot':
                temp_yaw, self.pitch, self.roll = [normalizeAnglePIPI(math.radians(x/100)) for x in data]
                if self.yaw_offset is None:
                    self.yaw_offset = -temp_yaw
                self.yaw = temp_yaw + self.yaw_offset

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
        dx, dy, __ = self.offset
        trace = Trace()
        trace.add_line_to((-dx, -dy, 0))
        if path is not None:
            for x, y in path:
                trace.add_line_to((x - dx, y - dy, 0))
        trace.reverse()
        self.follow_trace(trace, timeout=timedelta(seconds=120), max_target_distance=2.5, safety_limit=0.2)

    def robust_follow_wall(self, radius, right_wall=False, timeout=timedelta(hours=3), dist_limit=None, flipped=False,
            pitch_limit=None, roll_limit=None):
        """
        Handle multiple re-tries with increasing distance from the wall if necessary
        """
        allow_virtual_flip = self.symmetric
        walldist = self.walldist
        total_dist = 0.0
        start_time = self.sim_time_sec
        overall_timeout = timeout
        while self.sim_time_sec - start_time < overall_timeout.total_seconds():
            if self.sim_time_sec - start_time > overall_timeout.total_seconds():
                print('Total Timeout Reached', overall_timeout.total_seconds())
                break
            timeout = timedelta(seconds=overall_timeout.total_seconds() - (self.sim_time_sec - start_time))
            print('Current timeout', timeout)

            dist, reason = self.follow_wall(radius=walldist, right_wall=right_wall, timeout=timeout, flipped=flipped,
                                    pitch_limit=self.limit_pitch, roll_limit=self.limit_roll)
            total_dist += dist
            if reason is None or reason in [REASON_LORA,]:
                break

            walldist += 0.2
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
                self.follow_wall(radius=walldist, right_wall=not right_wall, timeout=timedelta(seconds=30), dist_limit=2.0,
                    flipped=not flipped, pitch_limit=self.return_limit_pitch, roll_limit=self.return_limit_roll)

                dist, reason = self.follow_wall(radius=walldist, right_wall=right_wall, timeout=timedelta(seconds=40), dist_limit=4.0,
                                        pitch_limit=self.limit_pitch, roll_limit=self.limit_roll, flipped=flipped)
                total_dist += dist
                if reason is None:
                    break
                if reason in [REASON_LORA, REASON_DIST_REACHED]:
                    break
                walldist += 0.2
            walldist = self.walldist
            if reason in [REASON_LORA,]:
                break


    def play_system_track(self):
        print("SubT Challenge Ver1!")
        try:
            with EmergencyStopMonitor(self):
                allow_virtual_flip = self.symmetric
                if distance(self.offset, (0, 0)) > 0.1 or self.init_path is not None:
                    self.system_nav_trace(self.init_path)

#                self.go_straight(2.5)  # go to the tunnel entrance - commented our for testing
                walldist = self.walldist
                total_dist = 0.0
                start_time = self.sim_time_sec
                while self.sim_time_sec - start_time < self.timeout.total_seconds():
                    if self.sim_time_sec - start_time > self.timeout.total_seconds():
                        print('Total Timeout Reached', self.timeout.total_seconds())
                        break
                    timeout = timedelta(seconds=self.timeout.total_seconds() - (self.sim_time_sec - start_time))
                    print('Current timeout', timeout)

                    dist, reason = self.follow_wall(radius=walldist, right_wall=self.use_right_wall, timeout=timeout,
                                            pitch_limit=self.limit_pitch, roll_limit=self.limit_roll)
                    total_dist += dist
                    if reason is None or reason in [REASON_LORA,]:
                        break

                    walldist += 0.2
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
                        self.follow_wall(radius=walldist, right_wall=not self.use_right_wall, timeout=timedelta(seconds=30), dist_limit=2.0,
                            flipped=allow_virtual_flip, pitch_limit=self.return_limit_pitch, roll_limit=self.return_limit_roll)

                        dist, reason = self.follow_wall(radius=walldist, right_wall=self.use_right_wall, timeout=timedelta(seconds=40), dist_limit=4.0,
                                                pitch_limit=self.limit_pitch, roll_limit=self.limit_roll)
                        total_dist += dist
                        if reason is None:
                            break
                        if reason in [REASON_LORA, REASON_DIST_REACHED]:
                            break
                        walldist += 0.2
                    walldist = self.walldist
                    if reason in [REASON_LORA,]:
                        break

                if self.use_return_trace and self.local_planner is not None:
                    self.stdout(self.time, "Going HOME %.3f" % dist, reason)
                    if allow_virtual_flip:
                        self.flipped = True  # return home backwards
                        self.scan = None
                    self.return_home(2 * self.timeout, home_threshold=1.0)
                    self.send_speed_cmd(0, 0)
                else:
                    print(self.time, "Going HOME", reason)
                    if not allow_virtual_flip:
                        self.turn(math.radians(90), timeout=timedelta(seconds=20))
                        self.turn(math.radians(90), timeout=timedelta(seconds=20))
                    self.robust_follow_wall(radius=self.walldist, right_wall=not self.use_right_wall, timeout=3*self.timeout, dist_limit=3*total_dist,
                            flipped=allow_virtual_flip, pitch_limit=self.return_limit_pitch, roll_limit=self.return_limit_roll)
                if self.artifacts:
                    self.bus.publish('artf_xyz', [[artifact_data, round(x*1000), round(y*1000), round(z*1000)]
                                              for artifact_data, (x, y, z) in self.artifacts])
        except EmergencyStopException:
            print(self.time, "EMERGENCY STOP - terminating")
        self.send_speed_cmd(0, 0)
        self.wait(timedelta(seconds=3))
#############################################

    def go_to_entrance(self):
        """
        Navigate to the base station tile end
        """
        dx, dy, dz = self.offset
        trace = Trace()  # starts by default at (0, 0, 0) and the robots are placed X = -7.5m (variable Y)
        trace.add_line_to((-4.5 - dx, -dy, self.height_above_ground))  # in front of the tunnel/entrance
        if self.use_right_wall:
            entrance_offset = -0.5
        elif self.use_center:
            entrance_offset = 0
        else:
            entrance_offset = 0.5
        trace.add_line_to((0.5 - dx, -dy + entrance_offset, dz + self.height_above_ground))  # 0.5m inside, towards the desired wall.
        trace.reverse()
        self.follow_trace(trace, timeout=timedelta(seconds=30), max_target_distance=2.5, safety_limit=0.2)

    def play_virtual_part(self):
        self.stdout("Waiting for origin ...")
        self.origin = None  # invalidate origin
        self.origin_error = False
        self.bus.publish('request_origin', True)
        while self.origin is None and not self.origin_error:
            self.update()
        self.stdout('Origin:', self.origin, self.robot_name)

        if self.origin is not None:
            x, y, z = self.origin
            x1, y1, z1 = self.xyz
            self.offset = x - x1, y - y1, z - z1
            self.stdout('Offset:', self.offset)
            heading = quaternion.heading(self.origin_quat)
            self.stdout('heading', math.degrees(heading), 'angle', math.degrees(math.atan2(-y, -x)), 'dist', math.hypot(x, y))

            self.go_to_entrance()
        else:
            # lost in tunnel
            self.stdout('Lost in tunnel:', self.origin_error, self.offset)

        start_time = self.sim_time_sec
        for loop in range(100):
            self.collision_detector_enabled = True
            if self.sim_time_sec - start_time > self.timeout.total_seconds():
                print('Total Timeout Reached', self.timeout.total_seconds())
                break
            timeout = timedelta(seconds=self.timeout.total_seconds() - (self.sim_time_sec - start_time))
            print('Current timeout', timeout)

            dist, reason = self.follow_wall(radius=self.walldist, right_wall=self.use_right_wall,  # was radius=0.9
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
                dist, reason = self.follow_wall(radius=self.walldist, right_wall=self.use_right_wall,  # was radius=0.9
                                    timeout=timedelta(seconds=60), pitch_limit=self.limit_pitch, roll_limit=None)
                self.use_center = before_center
                if reason is None or reason != REASON_PITCH_LIMIT:
                    continue

            if reason is None or reason != REASON_PITCH_LIMIT:
                break
            self.stdout(self.time, "Microstep HOME %d %.3f" % (loop, dist), reason)
            self.go_straight(-0.3, timeout=timedelta(seconds=10))
            self.return_home(timedelta(seconds=10))

        self.stdout("Artifacts:", self.artifacts)

        self.stdout(self.time, "Going HOME %.3f" % dist, reason)

        self.return_home(2 * self.timeout)
        self.send_speed_cmd(0, 0)

        if self.artifacts:
            self.bus.publish('artf_xyz', [[artifact_data, round(x*1000), round(y*1000), round(z*1000)] 
                                          for artifact_data, (x, y, z) in self.artifacts])

        self.wait(timedelta(seconds=10), use_sim_time=True)

    def play_virtual_track(self):
        self.stdout("SubT Challenge Ver73!")
        self.stdout("Waiting for robot_name ...")
        while self.robot_name is None:
            self.update()
        self.stdout('robot_name:', self.robot_name)

        # wait for critical data
        while any_is_none(self.scan, self.yaw_offset, self.xyz):
            # self.xyz is initialized by pose2d or pose3d depending on robot type
            self.update()

        if self.use_right_wall == 'auto':
            self.use_right_wall = self.robot_name.endswith('R')
            self.use_center = self.robot_name.endswith('C')
        self.stdout('Use right wall:', self.use_right_wall)

        times_sec = [int(x) for x in self.robot_name[1:-1].split('F')]
        self.stdout('Using times', times_sec)

        # add extra sleep to give a chance to the other robot (based on name)
        self.wait(timedelta(seconds=times_sec[0]), use_sim_time=True)

        # potential wrong artifacts:
        self.stdout('Artifacts before start:', self.artifacts)

        for timeout_sec in times_sec[1:]:
            self.timeout = timedelta(seconds=timeout_sec)
            self.play_virtual_part()
            self.stdout('Final xyz:', self.xyz)
            x, y, z = self.xyz
            x0, y0, z0 = self.offset
            self.stdout('Final xyz (DARPA coord system):', (x + x0, y + y0, z + z0))

        self.wait(timedelta(seconds=30), use_sim_time=True)
#        self.dumplog()
#        self.wait(timedelta(seconds=10), use_sim_time=True)

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
    parser_run.add_argument('--walldist', help='distance for wall following (default: %(default)sm)', default=1.0, type=float)
    parser_run.add_argument('--side', help='which side to follow', choices=['left', 'right', 'auto'], required=True)
    parser_run.add_argument('--speed', help='maximum speed (default: from config)', type=float)
    parser_run.add_argument('--timeout', help='seconds of exploring before going home (default: %(default)s)',
                            type=int, default=10*60)
    parser_run.add_argument('--log', nargs='?', help='record log filename')
    parser_run.add_argument('--init-offset', help='inital 3D offset accepted as a string of comma separated values (meters)')
    parser_run.add_argument('--init-path', help='inital path to be followed from (0, 0). 2D coordinates are separated by ;')
    parser_run.add_argument('--start-paused', dest='start_paused', action='store_true',
                            help='start robota Paused and wait for LoRa Contine command')

    parser_replay = subparsers.add_parser('replay', help='replay from logfile')
    parser_replay.add_argument('logfile', help='recorded log file')
    parser_replay.add_argument('--force', '-F', dest='force', action='store_true', help='force replay even for failing output asserts')
    parser_replay.add_argument('--config', nargs='+', help='force alternative configuration file')
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
        cfg['robot']['modules']['app']['init']['walldist'] = args.walldist
        if args.side == 'auto':
            cfg['robot']['modules']['app']['init']['right_wall'] = 'auto'
        else:
            cfg['robot']['modules']['app']['init']['right_wall'] = args.side == 'right'
        cfg['robot']['modules']['app']['init']['timeout'] = args.timeout
        if args.init_offset is not None:
            x, y, z = [float(x) for x in args.init_offset.split(',')]
            cfg['robot']['modules']['app']['init']['init_offset'] = [int(x*1000), int(y*1000), int(z*1000)]
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
