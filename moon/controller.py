"""
  Space Robotics Challenge 2
"""
import math
from random import Random
from datetime import timedelta
from statistics import median

from osgar.bus import BusShutdownException
from osgar.lib import quaternion
from osgar.lib.mathex import normalizeAnglePIPI
from osgar.lib.virtual_bumper import VirtualBumper

from subt.local_planner import LocalPlanner

from moon.moonnode import MoonNode


class ChangeDriverException(Exception):
    pass

class VirtualBumperException(Exception):
    pass


def distance(pose1, pose2):
    return math.hypot(pose1[0] - pose2[0], pose1[1] - pose2[1])


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
        self.scan_history = []
        self.threshold_distance = 1200 #1.2m
        self.min_hits = 10 # we have to see at least 10 points nearer than threshold

    def update(self, robot, channel):
        if channel == 'scan':
            # measure distance only in 66 degree angle (about the width of the robot 1.5m ahead)
            # NASA Lidar 150degrees wide, 100 samples
            # robot is ~2.21m wide (~1.2m x 2 with wiggle room)

            collision_view = robot.scan[70:110]
            self.scan_history.append(collision_view)
            if len(self.scan_history) > 3:
                self.scan_history.pop(0)

            median_scan = []
            for j in range(len(collision_view)):
                median_scan.append(median([self.scan_history[i][j] for i in range(len(self.scan_history))]))

            iterator = filter(lambda dist : 10 < dist < self.threshold_distance, median_scan)
            if  len(list(iterator)) >= self.min_hits and not robot.inException:
                robot.publish('driving_recovery', True)
                raise LidarCollisionException()

    # context manager functions
    def __enter__(self):
        self.callback = self.robot.register(self.update)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.robot.unregister(self.callback)


class SpaceRoboticsChallenge(MoonNode):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register("desired_speed", "artf_xyz", "artf_cmd", "pose2d", "pose3d", "driving_recovery", "request", "cmd")

        self.last_position = None
        self.max_speed = 1.0  # oficial max speed is 1.5m/s
        self.max_angular_speed = math.radians(60)

        self.min_safe_dist = config.get('min_safe_dist', 2.0)
        self.dangerous_dist = config.get('dangerous_dist', 1.5)
        self.safety_turning_coeff = config.get('safety_turning_coeff', 0.8)
        scan_subsample = config.get('scan_subsample', 1)
        obstacle_influence = config.get('obstacle_influence', 0.8)
        direction_adherence = math.radians(config.get('direction_adherence', 90))
        self.local_planner = LocalPlanner(
                obstacle_influence=obstacle_influence,
                direction_adherence=direction_adherence,
                max_obstacle_distance=4.0,
                scan_subsample=scan_subsample,
                max_considered_obstacles=100)

        self.origin = None  # unknown initial position
        self.origin_quat = quaternion.identity()
        self.start_pose = None
        self.yaw_offset = None
        self.yaw, self.pitch, self.roll = 0, 0, 0
        self.xyz = (0, 0, 0)  # 3D position for mapping artifacts
        self.xyz_quat = [0, 0, 0]
        self.offset = (0, 0, 0)
        self.use_gimbal = True # try to keep the camera on level as we go over obstacles

        self.brakes_on = False
        self.camera_change_triggered_time = None
        self.camera_angle = 0.0

        self.joint_name = None

        self.score = 0
        self.current_driver = None

        self.inException = False
        self.in_driving_recovery = False

        self.last_status_timestamp = None

        self.virtual_bumper = None
        self.rand = Random(0)

    def register(self, callback):
        self.monitors.append(callback)
        return callback

    def unregister(self, callback):
        assert callback in self.monitors
        self.monitors.remove(callback)

    def send_speed_cmd(self, speed, angular_speed):
        if self.virtual_bumper is not None:
            self.virtual_bumper.update_desired_speed(speed, angular_speed)
        self.bus.publish('desired_speed', [round(speed*1000), round(math.degrees(angular_speed)*100)])

    def send_request(self, cmd):
        """Send ROS Service Request form a single place"""
        self.publish('request', cmd)
        while True:
            dt, channel, data = self.listen()
            if channel == 'response':
                return data
            print(dt, 'ignoring', channel)

    def set_cam_angle(self, angle):
        self.send_request('set_cam_angle %f\n' % angle)
        self.camera_angle = angle
        print (self.sim_time, "app: Camera angle set to: %f" % angle)
        self.camera_change_triggered_time = self.sim_time

    def set_brakes(self, on):
        assert type(on) is bool, on
        self.brakes_on = on
        self.send_request('set_brakes %s\n' % ('on' if on else 'off'))
        print (self.sim_time, "app: Brakes set to: %s" % on)

    def on_driving_recovery(self, data):
        self.in_driving_recovery = data
        print (self.sim_time, "Driving recovery changed to: %r" % data)

    def on_pose2d(self, data):
        x, y, heading = data
        pose = (x / 1000.0, y / 1000.0, math.radians(heading / 100.0))
        if self.last_position is not None:
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
        x, y, z = self.xyz
        x += math.cos(self.pitch) * math.cos(self.yaw) * dist
        y += math.cos(self.pitch) * math.sin(self.yaw) * dist
        z += math.sin(self.pitch) * dist
        x0, y0, z0 = self.offset
        self.last_send_time = self.bus.publish('pose2d', [round((x + x0) * 1000), round((y + y0) * 1000),
                                    round(math.degrees(self.yaw) * 100)])
        self.xyz = x, y, z
        if self.virtual_bumper is not None:
            self.virtual_bumper.update_pose(self.sim_time, pose)
            if not self.inException and self.virtual_bumper.collision():
                self.bus.publish('driving_recovery', True)
                raise VirtualBumperException()

    def on_driving_control(self, data):
        # someone else took over driving
        self.current_driver = data

    def on_score(self, data):
        self.score = data[0]

    def on_scan(self, data):
        pass

    def on_joint_position(self, data):
        pass

    def on_rot(self, data):
        temp_yaw, self.pitch, self.roll = [normalizeAnglePIPI(math.radians(x/100)) for x in data]
        if self.yaw_offset is None:
            self.yaw_offset = -temp_yaw
        self.yaw = temp_yaw + self.yaw_offset

        if self.use_gimbal:
            # maintain camera level
            cam_angle = self.camera_angle + self.pitch
            self.publish('cmd', b'set_cam_angle %f' % cam_angle)

        if not self.inException and self.pitch < -0.6:
            # TODO pitch can also go the other way if we back into an obstacle
            # TODO: robot can also roll if it runs on a side of a rock while already on a slope
            self.bus.publish('driving_recovery', True)
            print (self.sim_time, "app: Excess pitch, going back down")
            raise VirtualBumperException()

    def update(self):

        # print status periodically - location
        if self.sim_time is not None:
            if self.last_status_timestamp is None:
                self.last_status_timestamp = self.sim_time
            elif self.sim_time - self.last_status_timestamp > timedelta(seconds=8):
                self.last_status_timestamp = self.sim_time
                x, y, z = self.xyz
                print (self.sim_time, "Loc: [%f %f %f] [%f %f %f]; Driver: %s; Score: %d" % (x, y, z, self.roll, self.pitch, self.yaw, self.current_driver, self.score))

        channel = super().update()
        return channel

    def go_straight(self, how_far, timeout=None):
        print(self.sim_time, "go_straight %.1f (speed: %.1f)" % (how_far, self.max_speed), self.last_position)
        start_pose = self.last_position
        if how_far >= 0:
            self.send_speed_cmd(self.max_speed, 0.0)
        else:
            self.send_speed_cmd(-self.max_speed, 0.0)
        start_time = self.sim_time
        while distance(start_pose, self.last_position) < abs(how_far):
            self.update()
            if timeout is not None and self.sim_time - start_time > timeout:
                print("go_straight - timeout at %.1fm" % distance(start_pose, self.last_position))
                break
        self.send_speed_cmd(0.0, 0.0)

    def turn(self, angle, with_stop=True, speed=0.0, timeout=None):
        print(self.sim_time, "turn %.1f" % math.degrees(angle))
        if angle >= 0:
            self.send_speed_cmd(speed, self.max_angular_speed)
        else:
            self.send_speed_cmd(speed, -self.max_angular_speed)
        start_time = self.sim_time
        # problem with accumulated angle

        sum_angle = 0.0
        prev_angle = self.yaw
        while abs(sum_angle) < abs(angle):
            self.update()
            sum_angle += normalizeAnglePIPI(self.yaw - prev_angle)
            prev_angle = self.yaw
            if timeout is not None and self.sim_time - start_time > timeout:
                print(self.sim_time, "turn - timeout at %.1fdeg" % math.degrees(sum_angle))
                break
        if with_stop:
            self.send_speed_cmd(0.0, 0.0)
            start_time = self.sim_time
            while self.sim_time - start_time < timedelta(seconds=2):
                self.update()
            print(self.sim_time, 'stop at', self.sim_time - start_time)

    def wait(self, dt):  # TODO refactor to some common class
        while self.sim_time is None:
            self.update()
        start_sim_time = self.sim_time
        while self.sim_time - start_sim_time < dt:
            self.update()

    def go_safely(self, desired_direction):
        safety, safe_direction = self.local_planner.recommend(desired_direction)
        desired_angular_speed = 0.9 * safe_direction
        size = len(self.scan)
        dist = min_dist(self.scan[size//3:2*size//3])
#        print(safe_direction, safety, dist)
        if dist < self.min_safe_dist:
            desired_speed = self.max_speed * (dist - self.dangerous_dist) / (self.min_safe_dist - self.dangerous_dist)
        else:
            desired_speed = self.max_speed
        desired_speed = desired_speed * (1.0 - self.safety_turning_coeff * min(self.max_angular_speed, abs(desired_angular_speed)) / self.max_angular_speed)
        self.send_speed_cmd(desired_speed, desired_angular_speed)
        return safety

    def random_walk(self, timeout):
        start_time = self.sim_time
        while self.sim_time - start_time < timeout:
            if self.update() == 'scan':
                self.go_safely(0.0)

        self.send_speed_cmd(0.0, 0.0)

    def try_step_around(self):
        self.turn(math.radians(90), timeout=timedelta(seconds=10))

        # recovered enough at this point to switch to another driver (in case you see cubesat while doing the 3m drive or the final turn)
        self.bus.publish('driving_recovery', False)

        self.go_straight(3.0, timeout=timedelta(seconds=20))
        self.turn(math.radians(-90), timeout=timedelta(seconds=10))

    def run(self):
        try:
            print('Wait for definition of last_position and yaw')
            while self.last_position is None or self.yaw is None:
                self.update()  # define self.sim_time
            print('done at', self.sim_time)


            last_walk_start = 0.0
            start_time = self.sim_time
            while self.sim_time - start_time < timedelta(minutes=40):
                additional_turn = 0
                last_walk_start = self.sim_time
                try:
                    self.virtual_bumper = VirtualBumper(timedelta(seconds=4), 0.1)
                    with LidarCollisionMonitor(self):
                        if self.current_driver is None and not self.brakes_on:
                            self.go_straight(50.0, timeout=timedelta(minutes=2))
                        else:
                            self.wait(timedelta(minutes=2)) # allow for self driving, then timeout
                    self.update()
                except ChangeDriverException as e:
                    continue

                except (VirtualBumperException, LidarCollisionException) as e:
                    self.inException = True
# TODO: crashes if an exception (e.g., excess pitch) occurs while handling an exception (e.g., virtual/lidar bump)
                    print(self.sim_time, repr(e))
                    last_walk_end = self.sim_time
                    self.virtual_bumper = None
                    self.go_straight(-2.0, timeout=timedelta(seconds=10))
                    if last_walk_end - last_walk_start > timedelta(seconds=20): # if we went more than 20 secs, try to continue a step to the left
                        self.try_step_around()
                    else:
                        self.bus.publish('driving_recovery', False)

                    self.inException = False

                    print ("Time elapsed since start of previous leg: %d sec" % (last_walk_end.total_seconds()-last_walk_start.total_seconds()))
                    if last_walk_end - last_walk_start > timedelta(seconds=20):
                        # if last step only ran short time before bumper, time for a large random turn
                        # if it ran long time, maybe worth trying going in the same direction
                        continue
                    additional_turn = 30

                    # next random walk direction should be between 30 and 150 degrees
                    # (no need to go straight back or keep going forward)
                    # if part of virtual bumper handling, add 30 degrees to avoid the obstacle more forcefully

                deg_angle = self.rand.randrange(30 + additional_turn, 150 + additional_turn)
                deg_sign = self.rand.randint(0,1)
                if deg_sign:
                    deg_angle = -deg_angle
                try:
                    self.virtual_bumper = VirtualBumper(timedelta(seconds=20), 0.1)
                    self.turn(math.radians(deg_angle), timeout=timedelta(seconds=30))
                except ChangeDriverException as e:
                    continue

                except VirtualBumperException:
                    self.inException = True
                    print(self.sim_time, "Turn Virtual Bumper!")
                    self.virtual_bumper = None
                    if self.current_driver is not None:
                        # probably didn't throw in previous turn but during self driving
                        self.go_straight(-2.0, timeout=timedelta(seconds=10))
                        self.try_step_around()
                    self.turn(math.radians(-deg_angle), timeout=timedelta(seconds=30))
                    self.inException = False
                    self.bus.publish('driving_recovery', False)
        except BusShutdownException:
            pass

# vim: expandtab sw=4 ts=4
