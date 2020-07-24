"""
  Space Robotics Challenge 2
"""
import math
import numpy as np

from random import Random
from datetime import timedelta
from statistics import median

from osgar.bus import BusShutdownException
from osgar.lib import quaternion
from osgar.lib.quaternion import euler_zyx, euler_to_quaternion

from osgar.lib.mathex import normalizeAnglePIPI
from osgar.lib.virtual_bumper import VirtualBumper

from subt.local_planner import LocalPlanner

from moon.moonnode import MoonNode

class ChangeDriverException(Exception):
    pass

class VirtualBumperException(Exception):
    pass

def eulerAnglesToRotationMatrix(theta) :
    # TODO: correctness of signs in R_x and R_y has not been verified
    R_x = np.array([[1,         0,                  0,                   0 ],
                    [0,         math.cos(theta[0]), math.sin(theta[0]), 0 ],
                    [0,         -math.sin(theta[0]), math.cos(theta[0]),  0 ],
                    [0,         0,                  0,                   1 ]
                    ])



    R_y = np.array([[math.cos(theta[1]),    0,      -math.sin(theta[1]), 0 ],
                    [0,                     1,      0,                  0 ],
                    [math.sin(theta[1]),   0,      math.cos(theta[1]), 0 ],
                    [0,                     0,      0,                  1 ]
                    ])

    R_z = np.array([[math.cos(theta[2]),    -math.sin(theta[2]),    0, 0],
                    [math.sin(theta[2]),    math.cos(theta[2]),     0, 0],
                    [0,                     0,                      1, 0],
                    [0,                     0,                      0, 1]
                    ])


    R = np.dot(R_z, np.dot( R_y, R_x ))

    return R

def translationToMatrix(v):
    return np.asmatrix(np.array(
        [[1, 0, 0, v[0]],
         [0, 1, 0, v[1]],
         [0, 0, 1, v[2]],
         [0,0,0,1]]
    ))

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
        bus.register("desired_speed", "driving_recovery", "request", "cmd")

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

        # best (fusion) values
        self.yaw, self.pitch, self.roll = 0, 0, 0
        self.xyz = None
        self.xyz_quat = None

        self.tf = {
            'vslam': {
                'trans_matrix': None,
                'latest_xyz': None,
                'latest_quat': None,
                'timestamp': None
            },
            'odo': {
                'trans_matrix': None,
                'latest_xyz': (0,0,0),
                'latest_quat': None,
                'timestamp': None
            }
        } # transformations from position sources (odometry, vslam) to robot coordinates

        self.use_gimbal = True # try to keep the camera on level as we go over obstacles
        self.yaw_history = []
        self.pitch_history = []
        self.roll_history = []

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

        self.requests = {}

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

    def on_response(self, data):
        token, response = data
        #print(self.sim_time, "controller:response received: token=%s, response=%s" % (token, response))
        callback = self.requests[token]
        self.requests.pop(token)
        if callback is not None:
            callback(response)

    def send_request(self, cmd, callback=None):
        """Send ROS Service Request from a single place"""
        token = hex(self.rand.getrandbits(128))
        self.requests[token] = callback
        #print(self.sim_time, "controller:send_request:token: %s, command: %s" % (token, cmd))
        self.publish('request', [token, cmd])

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

    def set_light_intensity(self, intensity):
        self.send_request('set_light_intensity %s\n' % intensity)
        print (self.sim_time, "app: Light intensity set to: %s" % intensity)

    def on_driving_recovery(self, data):
        self.in_driving_recovery = data
        print (self.sim_time, "Driving recovery changed to: %r" % data)

    def register_origin(self, message):
        print ("controller: origin received: %s" % message)
        if message.split()[0] == 'origin':
            origin = [float(x) for x in message.split()[1:]]
            initial_xyz = origin[:3]
            initial_quat = origin[3:]
            initial_rpy = euler_zyx(initial_quat) # note: this is not in roll, pitch, yaw order

            self.xyz = initial_xyz
            self.xyz_quat = initial_quat

            # note: if VSLAM is not tracking at time of register_origin call, the latest reported position will be inaccurate and VSLAM won't work
            if self.tf['vslam']['latest_quat'] is not None:
                latest_vslam_rpy = euler_zyx(self.tf['vslam']['latest_quat']) # will be rearranged after offset calculation
                vslam_rpy_offset = [a-b for a,b in zip(initial_rpy, latest_vslam_rpy)]
                vslam_rpy_offset = [vslam_rpy_offset[2], vslam_rpy_offset[1], vslam_rpy_offset[0]] #rearrange angles to correct RPY order
                print(self.sim_time, "VSLAM RPY offset: " + str(vslam_rpy_offset))
                rot_matrix = np.asmatrix(eulerAnglesToRotationMatrix(vslam_rpy_offset))

                vslam_xyz_offset = translationToMatrix(self.tf['vslam']['latest_xyz'])
                orig_xyz_offset = translationToMatrix(initial_xyz)

                self.tf['vslam']['trans_matrix'] = np.dot(orig_xyz_offset, np.dot(rot_matrix, vslam_xyz_offset.I))

            if self.tf['odo']['latest_quat'] is not None:
                latest_odo_rpy = euler_zyx(self.tf['odo']['latest_quat']) # will be rearranged after offset calculation
                odo_xyz_offset = [a-b for a,b in zip(initial_xyz, self.tf['odo']['latest_xyz'])]
                odo_rpy_offset = [a-b for a,b in zip(initial_rpy, latest_odo_rpy)]
                odo_rpy_offset = [odo_rpy_offset[2], odo_rpy_offset[1], odo_rpy_offset[0]] #rearrange angles to correct RPY order
                print(self.sim_time, "ODO RPY offset: " + str(odo_rpy_offset))
                rot_matrix = np.asmatrix(eulerAnglesToRotationMatrix(odo_rpy_offset))

                odo_xyz_offset = translationToMatrix(self.tf['odo']['latest_xyz'])
                orig_xyz_offset = translationToMatrix(initial_xyz)

                self.tf['odo']['trans_matrix'] = np.dot(orig_xyz_offset, np.dot(rot_matrix, odo_xyz_offset.I))



    def on_vslam_pose(self, data):
        if math.isnan(data[0][0]): # VSLAM not tracking
            if self.tf['vslam']['trans_matrix'] is not None and not self.inException: # it was tracking so it is lost, go back and re-acquire lock
                raise LidarCollisionException
            return

        self.tf['vslam']['latest_xyz'] = data[0]
        self.tf['vslam']['latest_quat'] = data[1]
        self.tf['vslam']['timestamp'] = self.sim_time

        #print("Internal VSLAM: " + str(self.tf['vslam']['latest_xyz']) + str(self.tf['vslam']['latest_quat']))
        if self.sim_time is None or self.last_position is None or self.yaw is None:
            return

        #print("VSLAM XYZ:" + str(self.tf['vslam']['latest_xyz']))
        #print("VSLAM RPY:" + str(euler_zyx(data[1])))

        if self.tf['vslam']['trans_matrix'] is not None:
            # calculate
            v = np.asmatrix(np.array([self.tf['vslam']['latest_xyz'][0], self.tf['vslam']['latest_xyz'][1], self.tf['vslam']['latest_xyz'][2], 1]))
            m = np.dot(self.tf['vslam']['trans_matrix'], v.T)
            self.xyz = [m[0,0], m[1,0], m[2,0]]
            # TODO: update quat also, will need to calculate using offset from the initial position
            # self.xyz_quat = ....
        else:
            # if tracking before transformation was establish, report in internal coordinate system
            # (difference of locations is used for the purposes of Virtual Bumpers)
            self.xyz = self.tf['vslam']['latest_xyz']

        #print ("[VSLAM] xyz: " + str(self.xyz))

        self.last_position = (self.xyz[0], self.xyz[1], self.yaw) # yaw from IMU, pose in global coordinates
        if self.virtual_bumper is not None:
            self.virtual_bumper.update_pose(self.sim_time, self.last_position)
            if not self.inException and self.virtual_bumper.collision():
                self.bus.publish('driving_recovery', True)
                raise VirtualBumperException()


    def on_odo_pose(self, data):
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
        x, y, z = self.tf['odo']['latest_xyz']
        x += math.cos(self.pitch) * math.cos(self.yaw) * dist
        y += math.cos(self.pitch) * math.sin(self.yaw) * dist
        z += math.sin(self.pitch) * dist
        self.tf['odo']['latest_xyz'] = x, y, z
        self.tf['odo']['latest_quat'] = euler_to_quaternion(self.yaw, self.pitch, self.roll)
        self.tf['odo']['timestamp'] = self.sim_time

        # if VSLAM is lost, keep providing location through odo
        # TODO: when VSLAM is up, should it override/refresh ODO pose?
        if self.tf['vslam']['timestamp'] is None or (self.sim_time - self.tf['vslam']['timestamp'] > timedelta(milliseconds=300)):
            self.last_position = pose
            self.xyz = (x, y, z)

            if self.virtual_bumper is not None:
                self.virtual_bumper.update_pose(self.sim_time, pose) # in rover coordinates
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
        # use of on_rot is deprecated, will be replaced by on_orientation
        # also, this filtering does not work when an outlier is presented while angles near singularity
        # e.g., values 0, 1, 359, 358, 120 will return 120
        temp_yaw, temp_pitch, temp_roll = [normalizeAnglePIPI(math.radians(x/100)) for x in data]

        self.yaw_history.append(temp_yaw)
        self.pitch_history.append(temp_pitch)
        self.roll_history.append(temp_roll)
        if len(self.yaw_history) > 5:
            self.yaw_history.pop(0)
            self.pitch_history.pop(0)
            self.roll_history.pop(0)

        self.yaw = median(self.yaw_history)
        self.pitch = median(self.pitch_history)
        self.roll = median(self.roll_history)

        if self.use_gimbal:
            # maintain camera level
            cam_angle = self.camera_angle + self.pitch
            self.publish('cmd', b'set_cam_angle %f' % cam_angle)

        if not self.inException and abs(self.pitch) > 0.6:
            # TODO pitch can also go the other way if we back into an obstacle
            # TODO: robot can also roll if it runs on a side of a rock while already on a slope
            self.bus.publish('driving_recovery', True)
            print (self.sim_time, "app: Excess pitch, going back")
            raise VirtualBumperException()

    def update(self):

        # print status periodically - location
        if self.sim_time is not None:
            if self.last_status_timestamp is None:
                self.last_status_timestamp = self.sim_time
            elif self.sim_time - self.last_status_timestamp > timedelta(seconds=8) and self.xyz is not None:
                self.last_status_timestamp = self.sim_time
                x, y, z = self.xyz
                pose_src = "ODO" if self.tf['vslam']['timestamp'] is None or self.sim_time - self.tf['vslam']['timestamp'] > timedelta(milliseconds=300) else "VSLAM"
                print (self.sim_time, "Loc[%s]: [%f %f %f] [%f %f %f]; Driver: %s; Score: %d" % (pose_src, x, y, z, self.roll, self.pitch, self.yaw, self.current_driver, self.score))


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
            #print(self.sim_time, 'stop at', self.sim_time - start_time)

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

        self.go_straight(5.0, timeout=timedelta(seconds=20))
        self.turn(math.radians(-90), timeout=timedelta(seconds=10))

    def wait_for_init(self):
        print('Wait for definition of last_position and yaw')
        while self.sim_time is None or self.last_position is None or self.yaw is None:
            self.update()
        print('done at', self.sim_time)

# vim: expandtab sw=4 ts=4
