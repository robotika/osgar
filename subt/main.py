"""
  SubT Challenge Version 1
"""
import gc
import os.path
import math
import threading

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


# safety limits for exploration and return home
LIMIT_ROLL = math.radians(25)  # in Virtual Urban are ramps with 20deg slope
LIMIT_PITCH = math.radians(25)
RETURN_LIMIT_ROLL = math.radians(35)
RETURN_LIMIT_PITCH = math.radians(35)

TRACE_STEP = 0.5  # meters in 3D

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


def min_dist(laser_data):
    if len(laser_data) > 0:
        # remove ultra near reflections and unlimited values == 0
        laser_data = [x if x > 10 else 10000 for x in laser_data]
        return min(laser_data)/1000.0
    return 0


def distance(pose1, pose2):
    return math.hypot(pose1[0] - pose2[0], pose1[1] - pose2[1])


def distance3D(xyz1, xyz2, weights=[1.0, 1.0, 1.0]):
    return math.sqrt(sum([w * (a-b)**2 for a, b, w in zip(xyz1, xyz2, weights)]))


class Trace:
    def __init__(self, step=TRACE_STEP):
        self.trace = [(0, 0, 0)]  # traveled 3D positions
        self.step = step

    def update_trace(self, pos_xyz):
        if distance3D(self.trace[-1], pos_xyz) >= self.step:
            self.trace.append(pos_xyz)

    def prune(self, radius=None):
        # use short-cuts and remove all cycles
        if radius is None:
            radius = self.step

        pruned = Trace(step=self.step)
        open_end = 1
        while open_end < len(self.trace):
            best = open_end
            for i, xyz in enumerate(self.trace[open_end:], start=open_end):
                if distance3D(xyz, pruned.trace[-1]) < radius:
                    best = i
            pruned.update_trace(self.trace[best])
            open_end = best + 1
        self.trace = pruned.trace


    def where_to(self, xyz, max_target_distance):
        # looking for a target point within max_target_distance nearest to the start
        for _ in range(8):
            for target in self.trace:
                if distance3D(target, xyz, [1.0, 1.0, 0.2]) < max_target_distance:
                    return target
            # if the robot deviated too far from the trajectory, we need to look for more distant target points
            max_target_distance *= 1.5
        # robot is crazy far from the trajectory
        assert(False)

    def reverse(self):
        self.trace.reverse()

    def add_line_to(self, xyz):
        last = self.trace[-1]
        size = distance3D(last, xyz)
        dx, dy, dz = xyz[0] - last[0], xyz[1] - last[1], xyz[2] - last[2]
        for i in range(1, int(size/self.step)):
            s = self.step * i/size
            self.trace.append((last[0] + s*dx, last[1] + s*dy, last[2] + s*dz))
        self.trace.append(xyz)


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

    # context manager functions
    def __enter__(self):
        self.callback = self.robot.register(self.update)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.robot.unregister(self.callback)


class SubTChallenge:
    def __init__(self, config, bus):
        self.bus = bus
        bus.register("desired_speed", "pose2d", "artf_xyz", "pose3d", "stdout", "request_origin")
        self.start_pose = None
        self.traveled_dist = 0.0
        self.time = None
        self.max_speed = config['max_speed']
        self.max_angular_speed = math.radians(20)
        self.walldist = config['walldist']
        self.timeout = timedelta(seconds=config['timeout'])
        self.symmetric = config['symmetric']  # is robot symmetric?
        self.virtual_bumper_sec = config.get('virtual_bumper_sec')
        virtual_bumper_sec = config.get('virtual_bumper_sec')
        self.virtual_bumper = None
        if virtual_bumper_sec is not None:
            virtual_bumper_radius = config.get('virtual_bumper_radius', 0.1)
            self.virtual_bumper = VirtualBumper(timedelta(seconds=virtual_bumper_sec), virtual_bumper_radius)

        self.last_position = (0, 0, 0)  # proper should be None, but we really start from zero
        self.xyz = (0, 0, 0)  # 3D position for mapping artifacts
        self.xyz_quat = [0, 0, 0]
        self.orientation = quaternion.identity()
        self.yaw, self.pitch, self.roll = 0, 0, 0
        self.is_moving = None  # unknown
        self.scan = None  # I should use class Node instead
        self.flipped = False  # by default use only front part
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

        self.last_send_time = None
        self.origin = None  # unknown initial position
        self.origin_quat = quaternion.identity()
        self.offset = (0, 0, 0)
        self.origin_error = False
        self.robot_name = None  # received with origin
        if self.is_virtual:
            self.local_planner = LocalPlanner()
        else:
            self.local_planner = None
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
            safe_direction = desired_direction
        else:
            safety, safe_direction = self.local_planner.recommend(desired_direction)
        desired_angular_speed = 1.2 * safe_direction
        size = len(self.scan)
        dist = min_dist(self.scan[size//3:2*size//3])
        if dist < 0.75:  # 2.0:
#            desired_speed = self.max_speed * (1.2/2.0) * (dist - 0.4) / 1.6
            desired_speed = self.max_speed * (dist - 0.2) / 0.55
        else:
            desired_speed = self.max_speed  # was 2.0
        '''
        desired_angular_speed = 0.7 * safe_direction
        T = math.pi / 2
        desired_speed = 2.0 * (0.8 - min(T, abs(desired_angular_speed)) / T)
        '''
        if self.flipped:
            self.send_speed_cmd(-desired_speed, desired_angular_speed)  # ??? angular too??!
        else:
            self.send_speed_cmd(desired_speed, desired_angular_speed)

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

                if self.lora_cmd is not None:
                    # the "GoHome" command must be accepted only on the way there and not on the return home
                    if dist_limit is None and self.lora_cmd == LORA_GO_HOME_CMD:
                        print(self.time, 'LoRa cmd - GoHome')
                        self.lora_cmd = None
                        reason = REASON_LORA
                        break
                    if self.lora_cmd == LORA_STOP_CMD:
                        print(self.time, 'LoRa cmd - Stop')
                        raise EmergencyStopException()
                    elif self.lora_cmd == LORA_PAUSE_CMD:
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

    def return_home(self, timeout):
        HOME_THRESHOLD = 5.0
        SHORTCUT_RADIUS = 2.3
        MAX_TARGET_DISTANCE = 5.0
        assert(MAX_TARGET_DISTANCE > SHORTCUT_RADIUS) # Because otherwise we could end up with a target point more distant from home than the robot.
        self.trace.prune(SHORTCUT_RADIUS)
        start_time = self.sim_time_sec
        while distance3D(self.xyz, (0, 0, 0)) > HOME_THRESHOLD and self.sim_time_sec - start_time < timeout.total_seconds():
            if self.update() == 'scan':
                target_x, target_y = self.trace.where_to(self.xyz, MAX_TARGET_DISTANCE)[:2]
                x, y = self.xyz[:2]
                desired_direction = math.atan2(target_y - y, target_x - x) - self.yaw
                self.go_safely(desired_direction)
        print('return_home: dist', distance3D(self.xyz, (0, 0, 0)), 'time(sec)', self.sim_time_sec - start_time)

    def follow_trace(self, trace, timeout, max_target_distance=5.0):
        print('Follow trace')
        END_THRESHOLD = 2.0
        start_time = self.sim_time_sec
        while distance3D(self.xyz, trace.trace[0]) > END_THRESHOLD and self.sim_time_sec - start_time < timeout.total_seconds():
            if self.update() == 'scan':
                target_x, target_y = trace.where_to(self.xyz, max_target_distance)[:2]
                x, y = self.xyz[:2]
#                print((x, y), (target_x, target_y))
                desired_direction = math.atan2(target_y - y, target_x - x) - self.yaw
                self.go_safely(desired_direction)
        print('End of follow trace(sec)', self.sim_time_sec - start_time)

    def register(self, callback):
        self.monitors.append(callback)
        return callback

    def unregister(self, callback):
        assert callback in self.monitors
        self.monitors.remove(callback)

    def on_pose2d(self, timestamp, data):
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
        if self.start_pose is None:
            self.start_pose = pose
        self.traveled_dist += dist
        x, y, z = self.xyz
        x += math.cos(self.pitch) * math.cos(self.yaw) * dist
        y += math.cos(self.pitch) * math.sin(self.yaw) * dist
        z += math.sin(self.pitch) * dist
        self.last_send_time = self.bus.publish('pose2d', [round(x * 1000), round(y * 1000),
                                    round(math.degrees(self.yaw) * 100)])
        if self.virtual_bumper is not None:
            self.virtual_bumper.update_pose(self.time, pose)  # sim time?!
        self.xyz = x, y, z
        self.trace.update_trace(self.xyz)
        # pose3d
        dist3d = quaternion.rotate_vector([dist, 0, 0], self.orientation)
        self.xyz_quat = [a + b for a, b in zip(self.xyz_quat, dist3d)]
        self.bus.publish('pose3d', [self.xyz_quat, self.orientation])

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

    def update(self):
        packet = self.bus.listen()
        if packet is not None:
#            print('SubT', packet)
            timestamp, channel, data = packet
            if self.time is None or int(self.time.seconds)//60 != int(timestamp.seconds)//60:
                self.stdout(timestamp, '(%.1f %.1f %.1f)' % self.xyz, sorted(self.stat.items()))
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
                    if self.ref_count > 30:
                        print('Robot is stuck!', self.ref_count)
                        if self.collision_detector_enabled:
                            self.collision_detector_enabled = False
                            raise Collision()
                        self.ref_count = 0

                if self.local_planner is not None:
                    if self.last_send_time is not None and self.last_send_time - self.time < timedelta(seconds=0.1):
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
                self.yaw, self.pitch, self.roll = [normalizeAnglePIPI(math.radians(x/100)) for x in data]
            elif channel == 'orientation':
                self.orientation = data
            elif channel == 'sim_time_sec':
                self.sim_time_sec = data
            elif channel == 'origin':
                if self.origin is None:  # accept only initial offset
                    self.robot_name = data[0].decode('ascii')
                    if len(data) == 8:
                        self.origin = data[1:4]
                        qx, qy, qz, qw = data[4:]
                        self.origin_quat = qx, qy, qz, qw  # quaternion
                    else:
                        self.stdout('Origin ERROR received')
                        self.origin_error = True
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
    def play_system_track(self):
        print("SubT Challenge Ver1!")
        try:
            with EmergencyStopMonitor(self):
                allow_virtual_flip = self.symmetric
#                self.go_straight(2.5)  # go to the tunnel entrance - commented our for testing
                walldist = self.walldist
                total_dist = 0.0
                start_time = self.sim_time_sec
                while self.sim_time_sec - start_time < self.timeout.total_seconds():
                    dist, reason = self.follow_wall(radius=walldist, right_wall=self.use_right_wall, timeout=self.timeout,
                                            pitch_limit=LIMIT_PITCH, roll_limit=LIMIT_ROLL)
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
                            flipped=allow_virtual_flip, pitch_limit=RETURN_LIMIT_PITCH, roll_limit=RETURN_LIMIT_ROLL)

                        dist, reason = self.follow_wall(radius=walldist, right_wall=self.use_right_wall, timeout=timedelta(seconds=40), dist_limit=4.0,
                                                pitch_limit=LIMIT_PITCH, roll_limit=LIMIT_ROLL)
                        total_dist += dist
                        if reason is None:
                            break
                        if reason in [REASON_LORA, REASON_DIST_REACHED]:
                            break
                        walldist += 0.2
                    walldist = self.walldist
                    if reason in [REASON_LORA,]:
                        break

                print(self.time, "Going HOME", reason)
                if not allow_virtual_flip:
                    self.turn(math.radians(90), timeout=timedelta(seconds=20))
                    self.turn(math.radians(90), timeout=timedelta(seconds=20))
                self.follow_wall(radius=self.walldist, right_wall=not self.use_right_wall, timeout=3*self.timeout, dist_limit=3*total_dist,
                        flipped=allow_virtual_flip, pitch_limit=RETURN_LIMIT_PITCH, roll_limit=RETURN_LIMIT_ROLL)
                if self.artifacts:
                    self.bus.publish('artf_xyz', [[artifact_data, round(x*1000), round(y*1000), round(z*1000)]
                                              for artifact_data, (x, y, z) in self.artifacts])
        except EmergencyStopException:
            print(self.time, "EMERGENCY STOP - terminating")
        self.send_speed_cmd(0, 0)
        self.wait(timedelta(seconds=3))
#############################################

    def test_nav_trace0(self):
        """
        Navigate to the station platform
        """
        trace = Trace()
        trace.add_line_to((3, -5, 0))
        trace.add_line_to((15, -5, 0))
        trace.add_line_to((15, 10, 0))
        trace.add_line_to((-23, 12, -3.267))
        trace.add_line_to((-25.656, 6.839, -3.267))
        trace.add_line_to((-36.762, 7.108, -3.267))
        trace.add_line_to((-37.582, -22.426, -4.505))
        trace.add_line_to((-27.098, -23.95, -4.505))  # TODO properly define z-value
        trace.add_line_to((-27.69, -31.209, -6.297))
        trace.add_line_to((-28.685, -32.144, -6.297))
        trace.add_line_to((-30, -32.144, -5.795))
        trace.add_line_to((-32, -32.144, -5.795))
        trace.add_line_to((-34, -32.144, -5.795))
        trace.reverse()
        self.follow_trace(trace, timeout=timedelta(seconds=180))

    def test_nav_trace1(self):
        """
        Navigate to rails
        """
        __, dy, __ = self.offset
        dy -= 5.000014
        trace = Trace()
        trace.add_line_to((3, -5 - dy, 0))  # before tunnel
        trace.add_line_to((15, -5 - dy, 0))  # inside tunnel
        trace.add_line_to((15, 10 - dy, 0))
        trace.add_line_to((-23, 12 - dy, -3.267))
        trace.add_line_to((-25.656, 6.839 - dy, -3.267))
        trace.add_line_to((-36.762, 7.108 - dy, -3.267))
        trace.add_line_to((-37.582, -22.426 - dy, -4.505))
        trace.add_line_to((-25.084, -26.688 - dy, -6.297))
        trace.add_line_to((-24.981, -36.925 - dy, -6.297))
        trace.add_line_to((-23, -36.925 - dy, -6.297))
        trace.add_line_to((-21, -36.925 - dy, -6.297))
        trace.add_line_to((-19, -39 - dy, -6.297))  # rails?

        # follow railway to the left
        trace.add_line_to((-18.303, -39.863 - dy, -6.297))
        trace.add_line_to((-1018.303, -39.863 - dy, -6.297))

        trace.reverse()
        self.follow_trace(trace, timeout=timedelta(seconds=300), max_target_distance=2.5)

    def test_nav_trace(self):
        """
        Navigate to the base station tile end
        """
        __, dy, __ = self.offset
        dy -= 5.000014
        trace = Trace()
        trace.add_line_to((3, -5 - dy, 0))  # before tunnel
        trace.add_line_to((15, -5 - dy, 0))  # inside tunnel
        trace.add_line_to((15, 10 - dy, 0))
        trace.add_line_to((-23, 12 - dy, -3.267))
        trace.add_line_to((-25.656, 6.839 - dy, -3.267))
        trace.add_line_to((-36.762, 7.108 - dy, -3.267))
        trace.add_line_to((-37.582, -22.426 - dy, -4.505))
        trace.add_line_to((-25.084, -26.688 - dy, -6.297))
        trace.reverse()
        self.follow_trace(trace, timeout=timedelta(seconds=120), max_target_distance=2.5)

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

            self.test_nav_trace()  # hacking - experiment
#            self.turn(normalizeAnglePIPI(math.atan2(-y, -x) - heading))
#            self.go_straight(math.hypot(x, y))  # go to the tunnel entrance
        else:
            # lost in tunnel
            self.stdout('Lost in tunnel:', self.origin_error, self.offset)
        for loop in range(3):
            self.collision_detector_enabled = True
            dist, reason = self.follow_wall(radius=self.walldist, right_wall=self.use_right_wall,  # was radius=0.9
                                timeout=self.timeout, pitch_limit=LIMIT_PITCH, roll_limit=None)
            self.collision_detector_enabled = False
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

    def dumplog(self):
        import os
        filename = self.bus.logger.filename  # deep hack
        self.stdout("Dump Log:", filename)
        size = statinfo = os.stat(filename).st_size
        self.stdout("Size:", size)
        with open(filename, 'rb') as f:
            for i in range(0, size, 100):
                self.stdout(i, f.read(100))
        self.stdout("Dump END")

    def play_virtual_track(self):
        self.stdout("SubT Challenge Ver42!")
        self.stdout("Waiting for robot_name ...")
        while self.robot_name is None:
            self.update()
        self.stdout('robot_name:', self.robot_name)

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

    def request_stop(self):
        self.bus.shutdown()

    def join(self):
        self.thread.join()


def main():
    import argparse
    from osgar.lib.config import config_load
    from osgar.record import record

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
        # To reduce latency spikes as described in https://morepypy.blogspot.com/2019/01/pypy-for-low-latency-systems.html.
        # Increased latency leads to uncontrolled behavior and robot either missing turns or hitting walls.
        # Disabled garbage collection needs to be paired with gc.collect() at place(s) that are not time sensitive.
        gc.disable()

        # support simultaneously multiple platforms
        prefix = os.path.basename(args.config[0]).split('.')[0] + '-'
        cfg = config_load(*args.config, application=SubTChallenge)

        # apply overrides from command line
        cfg['robot']['modules']['app']['init']['walldist'] = args.walldist
        if args.side == 'auto':
            cfg['robot']['modules']['app']['init']['right_wall'] = 'auto'
        else:
            cfg['robot']['modules']['app']['init']['right_wall'] = args.side == 'right'
        cfg['robot']['modules']['app']['init']['timeout'] = args.timeout
        if args.speed is not None:
            cfg['robot']['modules']['app']['init']['max_speed'] = args.speed

        cfg['robot']['modules']['app']['init']['start_paused'] = args.start_paused

        record(cfg, prefix)


if __name__ == "__main__":
    main()

# vim: expandtab sw=4 ts=4
