"""
  Space Robotics Challenge 2
"""
import math
from math import sqrt
import numpy as np
import io

from datetime import timedelta
from statistics import median

from osgar.bus import BusShutdownException
from osgar.lib import quaternion
from osgar.lib.quaternion import euler_zyx, euler_to_quaternion

from osgar.lib.mathex import normalizeAnglePIPI
from osgar.lib.virtual_bumper import VirtualBumper

from moon.moonnode import MoonNode

TURN_RADIUS = 8 # radius of circle when turning
AVOID_RADIUS = 4 # radius to use when going around an obstacle (this means it will not rush to go back to the same direction once it disappears off lidar)
GO_STRAIGHT = float("inf")
AVOIDANCE_DURATION = 3000 # milliseconds
AVOIDANCE_TURN_DURATION = 800

class MoonException(Exception):
    pass
class ChangeDriverException(MoonException):
    pass

class VSLAMLostException(MoonException):
    pass

class VSLAMDisabledException(MoonException):
    pass

class VSLAMEnabledException(MoonException):
    pass

class VSLAMFoundException(MoonException):
    pass

class VirtualBumperException(MoonException):
    pass

class LidarCollisionException(MoonException):
    pass

def pol2cart(rho, phi):
    x = rho * math.cos(phi)
    y = rho * math.sin(phi)
    return(x, y)

def cart2pol(x, y):
    rho = np.sqrt(x**2 + y**2)
    phi = np.arctan2(y, x)
    return(rho, phi)

def best_fit_circle(x_l, y_l):
    # Best Fit Circle https://goodcalculators.com/best-fit-circle-least-squares-calculator/
    # receive 180 scan samples, first and last 40 are discarded, remaining 100 samples represent 2.6rad view
    # samples are supposed to form a circle which this routine calculates

    nop = len(x_l)
    x = np.array(x_l)
    y = np.array(y_l)

    x_y = np.multiply(x,y)
    x_2 = np.square(x)
    y_2 = np.square(y)

    x_2_plus_y_2 = np.add(x_2,y_2)
    x__x_2_plus_y_2 = np.multiply(x,x_2_plus_y_2)
    y__x_2_plus_y_2 = np.multiply(y,x_2_plus_y_2)

    sum_x = x.sum(dtype=float)
    sum_y = y.sum(dtype=float)
    sum_x_2 = x_2.sum(dtype=float)
    sum_y_2 = y_2.sum(dtype=float)
    sum_x_y = x_y.sum(dtype=float)
    sum_x_2_plus_y_2 = x_2_plus_y_2.sum(dtype=float)
    sum_x__x_2_plus_y_2 = x__x_2_plus_y_2.sum(dtype=float)
    sum_y__x_2_plus_y_2 = y__x_2_plus_y_2.sum(dtype=float)

    m3b3 = np.array([[sum_x_2,sum_x_y,sum_x],
            [sum_x_y,sum_y_2,sum_y],
            [sum_x,sum_y,nop]])
    invm3b3 = np.linalg.inv(m3b3)
    m3b1 = np.array([sum_x__x_2_plus_y_2,sum_y__x_2_plus_y_2,sum_x_2_plus_y_2])
    A=np.dot(invm3b3,m3b1)[0]
    B=np.dot(invm3b3,m3b1)[1]
    C=np.dot(invm3b3,m3b1)[2]
    homebase_cx = A/2
    homebase_cy = B/2
    homebase_radius = np.sqrt(4*C+A**2+B**2)/2

    return(homebase_cx, homebase_cy, homebase_radius)


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

def calc_tangents(C,r,P):
    dx, dy = P[0]-C[0], P[1]-C[1]
    dxr, dyr = -dy, dx
    d = sqrt(dx**2+dy**2)
    if d >= r :
        rho = r/d
        ad = rho**2
        bd = rho*sqrt(1-rho**2)
        T1 = [C[0] + ad*dx + bd*dxr, C[1] + ad*dy + bd*dyr]
        T2 = [C[0] + ad*dx - bd*dxr, C[1] + ad*dy - bd*dyr]

        if (d/r-1) < 1E-8:
            print('controller.calc_tangents: P is on the circumference')
        return [T1, T2]
    print('controller.calc_tangents: P inside circle')


def min_dist(laser_data):
    if len(laser_data) > 0:
        # remove ultra near reflections and unlimited values == 0
        laser_data = [x if x > 10 else 10000 for x in laser_data]
        return min(laser_data)/1000.0
    return 0


class LidarCollisionMonitor:
    def __init__(self, robot, threshold_distance=1200):
        self.robot = robot
        self.scan_history = []
        self.threshold_distance = threshold_distance #1.2m
        self.min_hits = 10 # we have to see at least 10 points nearer than threshold

    def update(self, robot, channel):
        if channel == 'scan':
            # measure distance only in 66 degree angle (about the width of the robot 1.5m ahead)
            # NASA Lidar 150degrees wide, 100 samples
            # robot is ~2.21m wide (~1.2m x 2 with wiggle room)
            # TODO: rework using lidar processing from main class

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

def ps(*args, **kwargs):
    output = io.StringIO()
    print(*args, file=output, **kwargs)
    contents = output.getvalue()
    output.close()
    return contents

class SpaceRoboticsChallenge(MoonNode):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register("desired_speed", "desired_movement", "driving_recovery", "cmd", "pose2d")
        self.robot_name = "scout_1"

        self.joint_name = None
        self.sensor_joint_position = None

        self.last_position = None # 2D pose (x, y, heading) in rover coordinates, used for local operations (go 10m)
        self.max_speed = 1.0  # oficial max speed is 1.5m/s
        self.max_angular_speed = math.radians(60)

        # best (fusion) values
        self.yaw, self.pitch, self.roll = 0, 0, 0
        self.xyz = None
        self.xyz_quat = None
        self.yaw_offset = 0

        # obstacle distance toolkit
        self.scan_distance_to_obstacle = 15000 # 15m min distance in front of robot
        self.scan_avg_distance_left = 15000
        self.scan_avg_distance_right = 15000
        self.scan_history = []
        self.median_scan = []
        self.scan_min_window = 10 # we have to see at least 10 points nearer than threshold
        self.scan_nr_kept = 3 # we have to see at least 10 points nearer than threshold

        self.avoidance_start = None
        self.avoidance_turn = None
        self.steering_angle = 0.0

        self.default_effort_level = 1000 # default is max speed

        self.last_reset_model = None # last time we reset the model (straighten up, drop from 1m)

        self.true_pose = False
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

    def register(self, callback):
        self.monitors.append(callback)
        return callback

    def unregister(self, callback):
        assert callback in self.monitors
        self.monitors.remove(callback)

    def send_speed_cmd(self, speed, angular_speed):
        self.bus.publish('desired_speed', [round(speed*1000), round(math.degrees(angular_speed)*100)])

    def set_cam_angle(self, angle):
        angle = min(math.pi / 4.0, max(-math.pi / 8.0, angle))
        self.send_request('set_cam_angle %f\n' % angle)
        self.camera_angle = angle
        print (self.sim_time, "app: Camera angle set to: %f" % angle)
        self.camera_change_triggered_time = self.sim_time

    def set_brakes(self, on):
        assert type(on) is bool, on
        self.brakes_on = on
        self.send_request('set_brakes %s\n' % ('on' if on else 'off'))
        print (self.sim_time, self.robot_name, "app: Brakes set to: %s" % on)

    def set_light_intensity(self, intensity):
        self.send_request('set_light_intensity %s\n' % intensity)
        print (self.sim_time, self.robot_name, "app: Light intensity set to: %s" % intensity)

    def on_driving_recovery(self, data):
        self.in_driving_recovery = data
        print (self.sim_time, self.robot_name, "Driving recovery changed to: %r" % data)

    def register_origin(self, message):
        print (self.sim_time, self.robot_name, "controller: origin received: %s" % (message))
        if message.split()[0] == 'origin':
            origin = [float(x) for x in message.split()[1:]]
            initial_xyz = origin[:3]
            initial_quat = origin[3:]
            initial_rpy = euler_zyx(initial_quat) # note: this is not in roll, pitch, yaw order

            self.xyz = initial_xyz
            self.xyz_quat = initial_quat
            self.yaw_offset = self.yaw - initial_rpy[0]

            for k, obj in self.tf.items():
            # note: if VSLAM is not tracking at time of register_origin call, the latest reported position will be inaccurate and VSLAM won't work
                if obj['latest_quat'] is not None:
                    latest_rpy = euler_zyx(obj['latest_quat']) # will be rearranged after offset calculation
                    rpy_offset = [a-b for a,b in zip(initial_rpy, latest_rpy)]
                    rpy_offset.reverse()
                    print(self.sim_time, self.robot_name, "%s RPY offset: %s" % (k, str(rpy_offset)))
                    rot_matrix = np.asmatrix(eulerAnglesToRotationMatrix(rpy_offset))

                    xyz_offset = translationToMatrix(obj['latest_xyz'])
                    orig_xyz_offset = translationToMatrix(initial_xyz)

                    obj['trans_matrix'] = np.dot(orig_xyz_offset, np.dot(rot_matrix, xyz_offset.I))
            self.true_pose = True



    def calculate_best_pose(self):
        # if VSLAM active, report its position either in internal or global coordinates depending on whether origin was established yet
        # otherwise use ODO position (internal or global)

        if self.tf['vslam']['timestamp'] is not None and self.sim_time - self.tf['vslam']['timestamp'] < timedelta(milliseconds=300):
            if self.tf['vslam']['trans_matrix'] is not None:
                v = np.asmatrix(np.array([self.tf['vslam']['latest_xyz'][0], self.tf['vslam']['latest_xyz'][1], self.tf['vslam']['latest_xyz'][2], 1]))
                m = np.dot(self.tf['vslam']['trans_matrix'], v.T)
                self.xyz = [m[0,0], m[1,0], m[2,0]]

                # use VSLAM pose to update ODO pose
                v = np.asmatrix(np.asarray([self.xyz[0], self.xyz[1], self.xyz[2], 1]))
                odo =  np.dot(self.tf['odo']['trans_matrix'].I, v.T)
                self.tf['odo']['latest_xyz'] = [odo[0,0], odo[1,0], odo[2,0]]

            else:
                self.xyz = self.tf['odo']['latest_xyz']

        else:
            if self.tf['odo']['trans_matrix'] is not None:
                v = np.asmatrix(np.array([self.tf['odo']['latest_xyz'][0], self.tf['odo']['latest_xyz'][1], self.tf['odo']['latest_xyz'][2], 1]))
                m = np.dot(self.tf['odo']['trans_matrix'], v.T)
                self.xyz = [m[0,0], m[1,0], m[2,0]]
            else:
                self.xyz = self.tf['odo']['latest_xyz']


        self.publish("pose2d", [round(1000*self.xyz[0]), round(1000*self.xyz[1]), round(100*math.degrees(self.yaw))])

        if self.virtual_bumper is not None:
            self.virtual_bumper.update_pose(self.sim_time, (self.xyz[0], self.xyz[1], self.yaw))
            if not self.inException and self.virtual_bumper.collision():
                self.bus.publish('driving_recovery', True)
                raise VirtualBumperException()

    def on_desired_speeds(self, data):
        linear_speed, angular_speed, steering_angle = data
        self.steering_angle = steering_angle
        if self.virtual_bumper is not None:
            self.virtual_bumper.update_desired_speed(linear_speed, angular_speed)

    def on_vslam_enabled(self, data):
        pass

    def on_vslam_pose(self, data):
        if self.sim_time is None or self.last_position is None or self.yaw is None:
            return
        if math.isnan(data[0][0]): # VSLAM not tracking
            return

        self.tf['vslam']['latest_xyz'] = data[0]
        self.tf['vslam']['latest_quat'] = data[1]
        self.tf['vslam']['timestamp'] = self.sim_time

        #print("Internal VSLAM: " + str(self.tf['vslam']['latest_xyz']) + str(self.tf['vslam']['latest_quat']))
        #print("VSLAM XYZ:" + str(self.tf['vslam']['latest_xyz']))
        #print("VSLAM RPY:" + str(euler_zyx(data[1])))

        self.calculate_best_pose()

    def on_odo_pose(self, data):
        x, y, heading = data
        pose = (x / 1000.0, y / 1000.0, math.radians(heading / 100.0)) # TODO: use IMU instead of wheels
        if self.last_position is not None:
            dist = distance(pose, self.last_position)
            direction = ((pose[0] - self.last_position[0]) * math.cos(self.last_position[2]) +
                         (pose[1] - self.last_position[1]) * math.sin(self.last_position[2]))
            if direction < 0:
                dist = -dist
        else:
            dist = 0.0
        self.last_position = pose

        x, y, z = self.tf['odo']['latest_xyz']
        x += math.cos(self.pitch) * math.cos(pose[2]) * dist
        y += math.cos(self.pitch) * math.sin(pose[2]) * dist
        z += math.sin(self.pitch) * dist
        self.tf['odo']['latest_xyz'] = x, y, z
        self.tf['odo']['latest_quat'] = euler_to_quaternion(self.yaw, self.pitch, self.roll)
        self.tf['odo']['timestamp'] = self.sim_time

        self.calculate_best_pose()


    def on_driving_control(self, data):
        # someone else took over driving
        self.current_driver = data

    def on_score(self, data):
        self.score = data[0]

    def on_scan(self, data):
        # data is 180 samples, first and last 40 are 0
        # NASA Lidar 150degrees wide, 100 samples
        # robot is ~2.21m wide (~1.2m x 2 with wiggle room)
        # 40 samples represents 60 degrees (1.0472rad)

        IDEAL_SAMPLE_SIZE = 30 # number of samples on each side of midpoint
        collision_view = [x if x > 10 else 15000 for x in data]
        self.scan_history.append(collision_view)
        if len(self.scan_history) > self.scan_nr_kept:
            self.scan_history.pop(0)
        else:
            return

        median_scan = []
        for j in range(len(collision_view)):
            median_scan.append(median([self.scan_history[i][j] for i in range(len(self.scan_history))]))
        self.median_scan = median_scan

        # measure distance only in 66 degree angle (about the width of the robot 1.5m)
        # however, don't look just ahead (camera angle) or direction of wheels (steering angle)
        # look further in the direction wheels are turned
        midpoint = int(max(50, min(130, 40 + 50 + 2 * self.steering_angle / 0.0262626260519)))
        if midpoint < 40 + IDEAL_SAMPLE_SIZE:
            sample_size = midpoint - 40
        elif midpoint > 140 - IDEAL_SAMPLE_SIZE:
            sample_size = 140 - midpoint
        else:
            sample_size = IDEAL_SAMPLE_SIZE

        distances = median_scan[midpoint:midpoint + sample_size]
        mean = np.mean(distances)
        sd = np.std(distances)
        d_clean = [x for x in distances if mean - 2 * sd < x < mean + 2 * sd]
        #self.scan_avg_distance_left = sum(d_clean) / len(d_clean) if len(d_clean) > 1 else 15000
        self.scan_avg_distance_left = min(d_clean) if len(d_clean) > 1 else 15000

        distances = median_scan[midpoint - sample_size:midpoint]
        mean = np.mean(distances)
        sd = np.std(distances)
        d_clean = [x for x in distances if mean - 2 * sd < x < mean + 2 * sd]
        #self.scan_avg_distance_right = sum(d_clean) / len(d_clean) if len(d_clean) > 1 else 15000
        self.scan_avg_distance_right = min(d_clean) if len(d_clean) > 1 else 15000

        #self.scan_avg_distance_left = sum(median_scan[midpoint:midpoint + sample_size]) / sample_size
        #self.scan_avg_distance_right = sum(median_scan[midpoint - sample_size:midpoint]) / sample_size
        before_robot = median_scan[midpoint - sample_size:midpoint + sample_size]
        before_robot.sort();
        self.scan_distance_to_obstacle = median(before_robot[:self.scan_min_window])

    def on_joint_position(self, data):
        assert self.joint_name is not None
        self.sensor_joint_position = data[self.joint_name.index(b'sensor_joint')]

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

        self.yaw = normalizeAnglePIPI(median(self.yaw_history) - self.yaw_offset)
        self.pitch = median(self.pitch_history)
        self.roll = median(self.roll_history)

        if self.use_gimbal:
            # maintain camera level
            cam_angle = min(math.pi / 4.0, max(-math.pi / 8.0, self.camera_angle + self.pitch))
            self.publish('cmd', b'set_cam_angle %f' % cam_angle)

        if self.virtual_bumper is not None and not self.inException and (abs(self.pitch) > 0.9 or abs(self.roll) > math.pi/4):
            # TODO pitch can also go the other way if we back into an obstacle
            # TODO: robot can also roll if it runs on a side of a rock while already on a slope
            self.bus.publish('driving_recovery', True)
            print (self.sim_time, self.robot_name, "app: Excess pitch or roll, going back")
            raise VirtualBumperException()

        if abs(self.roll) > math.pi/2 and (self.last_reset_model is None or self.sim_time - self.last_reset_model > timedelta(seconds=15)):
            # if roll is more than 90deg, robot is upside down
            self.last_reset_model = self.sim_time
            # self.send_request('reset_model')

    def get_extra_status(self):
        return ""

    def update(self):

        # print status periodically - location
        if self.sim_time is not None:
            if self.last_status_timestamp is None:
                self.last_status_timestamp = self.sim_time
            elif self.sim_time - self.last_status_timestamp > timedelta(seconds=8) and self.xyz is not None:
                self.last_status_timestamp = self.sim_time
                x, y, z = self.xyz
                s = ps("---------------------------------------------------------")
                s += ps(self.sim_time, "%s:" % self.robot_name)
                s += ps("RPY: [%f %f %f]; Driver: %s; Score: %d" % (self.roll, self.pitch, self.yaw, self.current_driver, self.score))
                s += ps("Loc[best]: [%f %f %f]" % (x, y, z))
                for k, obj in self.tf.items():
                    if obj['trans_matrix'] is not None and self.sim_time - obj['timestamp'] < timedelta(milliseconds=300):
                        x, y, z = obj['latest_xyz']
                        v = np.asmatrix(np.array([x, y, z, 1]))
                        m = np.dot(obj['trans_matrix'], v.T)
                        x,y,z = [m[0,0], m[1,0], m[2,0]]
                        s += ps("Loc[%s]: [%f %f %f]" % (k, x, y, z))
                s += self.get_extra_status()
                s += ("\n---------------------------------------------------------")
                print(s)

        channel = super().update()
        return channel

    def get_angle_diff(self, destination, direction=1):
        angle_diff = normalizeAnglePIPI(math.atan2(destination[1] - self.xyz[1], destination[0] - self.xyz[0]) - self.yaw)
        if direction < 0:
            angle_diff = - math.pi + angle_diff

        return normalizeAnglePIPI(angle_diff)

    def get_avoidance_turn(self):
        turn = None

        # if avoidance expired, reset turn
        if self.avoidance_start is not None and self.sim_time - self.avoidance_start >= timedelta(milliseconds=AVOIDANCE_DURATION):
            self.avoidance_turn = None

        # finish existing avoidance
        if self.avoidance_start is not None:
            if self.sim_time - self.avoidance_start < timedelta(milliseconds=AVOIDANCE_DURATION):
                if self.sim_time - self.avoidance_start < timedelta(milliseconds=AVOIDANCE_TURN_DURATION):
                    if self.debug:
                        print(self.sim_time, self.robot_name, "Still turning")
                    turn = self.avoidance_turn # keep turning 0.5s after obstacle no longer visible

                elif self.sim_time - self.avoidance_start < timedelta(milliseconds=AVOIDANCE_DURATION):
                    if self.debug:
                        print(self.sim_time, self.robot_name, "Going straight")
                    turn = GO_STRAIGHT # go straight additional 1s after no longer turning
            else:
                self.avoidance_turn = None

        if self.scan_distance_to_obstacle < 4000:
            # optionally do not avoid obstacles when real close to destination - if destination is a volatile, you know the obstacle is just a bump
            self.avoidance_start = self.sim_time
            if self.scan_avg_distance_left < 0.8 * self.scan_avg_distance_right:
                self.avoidance_turn = -AVOID_RADIUS # steer to the right
            elif self.scan_avg_distance_left * 0.8 > self.scan_avg_distance_right:
                self.avoidance_turn = AVOID_RADIUS
            else:
                if self.avoidance_turn is None:
                    self.avoidance_turn = AVOID_RADIUS if self.rand.getrandbits(1) == 0 else -AVOID_RADIUS
            turn = self.avoidance_turn
            if (self.debug):
                print(self.sim_time, self.robot_name, "Seeing object, turning: %d, distance %d" % (turn, self.scan_distance_to_obstacle))

        return turn


    def go_to_location(self, pos, desired_speed, offset=0.0, full_turn=False, timeout=None, with_stop=True, tolerance=1.0, avoid_obstacles_close_to_destination=False):
        # speed: + forward, - backward
        # offset: stop before (-) or past (+) the actual destination (e.g., to keep the destination in front of the robot)
        print(self.sim_time, self.robot_name, "go_to_location [%.1f,%.1f] + offset %.1f (speed: %.1f)" % (pos[0], pos[1], offset, math.copysign(self.max_speed, desired_speed)))

        dist = distance(pos, self.xyz)+offset
        if timeout is None:
            timeout = timedelta(seconds=(10 if full_turn else 0) + 2*dist / self.max_speed) # 30m should take at most 60 seconds, unless turning heavily, then add the amount of time it takes to turn around

        start_time = self.sim_time

        # while we are further than tolerance or while the distance from target is decreasing but still following the right direction (angle diff < 90deg)
        angle_diff = self.get_angle_diff(pos,desired_speed)
        last_dist = distance(pos, self.xyz) + offset
        while (distance(pos, self.xyz)+offset > tolerance or distance(pos, self.xyz)+offset < last_dist) and (full_turn or abs(angle_diff) < math.pi/2):
            last_dist = distance(pos, self.xyz)+offset
            if last_dist < 0:
                break
            angle_diff = self.get_angle_diff(pos,desired_speed)
            speed = desired_speed if last_dist > 4 or not with_stop else 0.6 * desired_speed

            if self.debug:
                print(self.sim_time, self.robot_name, "Distance ahead: %.1f, Left avg distance: %.1f, Right avg distance: %.1f" % (self.scan_distance_to_obstacle, self.scan_avg_distance_left, self.scan_avg_distance_right))

            turn = None
            if (
                    speed > 0 and
                    (avoid_obstacles_close_to_destination or last_dist > 5)
            ):
                turn = self.get_avoidance_turn()

            if turn is None:
                if angle_diff > 0.1 and last_dist > 1:
                    turn = TURN_RADIUS * math.copysign(1, speed)
                elif angle_diff < -0.1 and last_dist > 1:
                    turn = -TURN_RADIUS * math.copysign(1, speed)
                else:
                    turn = GO_STRAIGHT
            self.publish("desired_movement", [turn, 0, speed])

            self.wait(timedelta(milliseconds=50))
            angle_diff = self.get_angle_diff(pos,speed) # update angle again after wait just before the while loop

            if timeout is not None and self.sim_time - start_time > timeout:
                print(self.sim_time, self.robot_name, "go_to_location timeout ended at [%.1f,%.1f]" % (self.xyz[0], self.xyz[1]))
                break
        if with_stop:
            self.send_speed_cmd(0.0, 0.0)
        print(self.sim_time, self.robot_name, "go_to_location [%.1f,%.1f] ended at [%.1f,%.1f], yaw=%.2f, angle_diff=%f" % (pos[0], pos[1], self.xyz[0], self.xyz[1], self.yaw, angle_diff))

    def move_sideways(self, how_far, view_direction=None, timeout=None): # how_far: left is positive, right is negative
        print(self.sim_time, self.robot_name, "move_sideways %.1f (speed: %.1f)" % (how_far, self.max_speed), self.last_position)
        if timeout is None:
            timeout = timedelta(seconds=4 + 2*abs(how_far) / self.max_speed) # add 4 sec for wheel setup
        if view_direction is not None:
            angle_diff = normalizeAnglePIPI(view_direction - self.yaw)
            if abs(angle_diff) > math.radians(10): # only turn if difference more than 10deg
                self.turn(angle_diff, timeout=timedelta(seconds=10))

        start_pose = self.xyz
        start_time = self.sim_time
        while distance(start_pose, self.xyz) < abs(how_far):
            self.publish("desired_movement", [GO_STRAIGHT, -9000, -math.copysign(self.default_effort_level, how_far)])
            self.wait(timedelta(milliseconds=100))
            if timeout is not None and self.sim_time - start_time > timeout:
                print(self.sim_time, self.robot_name, "go_sideways - timeout at %.1fm" % distance(start_pose, self.xyz))
                break
        self.send_speed_cmd(0.0, 0.0)
        print(self.sim_time, self.robot_name, "move sideways ended at [%.1f,%.1f]" % (self.xyz[0], self.xyz[1]))


    def go_straight(self, how_far, with_stop=True, timeout=None):
        print(self.sim_time, self.robot_name, "go_straight %.1f (speed: %.1f)" % (how_far, self.max_speed), self.last_position)
        if timeout is None:
            timeout = timedelta(seconds=2*abs(how_far) / self.max_speed)

        start_pose = self.last_position
        if how_far >= 0:
            self.send_speed_cmd(self.max_speed, 0.0)
        else:
            self.send_speed_cmd(-self.max_speed, 0.0)
        start_time = self.sim_time
        slowdown_happened = False
        while distance(start_pose, self.last_position) < abs(how_far):
            if with_stop and not slowdown_happened and distance(start_pose, self.last_position) - abs(how_far) < 2:
                self.send_speed_cmd(math.copysign(self.max_speed / 2.0, how_far), 0.0)
                slowdown_happened = True

            self.update()
            if timeout is not None and self.sim_time - start_time > timeout:
                print(self.sim_time, self.robot_name, "go_straight - timeout at %.1fm" % distance(start_pose, self.last_position))
                break
        self.send_speed_cmd(0.0, 0.0)

    def turn(self, angle, with_stop=True, speed=0.0, ang_speed=None, timeout=None):
        # positive turn - counterclockwise
        print(self.sim_time, self.robot_name, "turn %.1f deg from %.1f deg" % (math.degrees(angle), math.degrees(self.yaw)))
        self.send_speed_cmd(speed, math.copysign(ang_speed if ang_speed is not None else self.max_angular_speed, angle))

        start_time = self.sim_time
        # problem with accumulated angle

        sum_angle = 0.0
        prev_angle = self.yaw
        slowdown_happened = False
        while abs(sum_angle) < abs(angle):
            if with_stop and not slowdown_happened and abs(sum_angle) > abs(angle) - 0.35: # slow rotation down for the last 20deg to improve accuracy
                self.send_speed_cmd(speed, math.copysign(self.max_angular_speed / 2.0, angle))
                slowdown_happened = True

            self.update()
            sum_angle += normalizeAnglePIPI(self.yaw - prev_angle)
            prev_angle = self.yaw
            if timeout is not None and self.sim_time - start_time > timeout:
                print(self.sim_time, self.robot_name, "turn - timeout at %.1fdeg" % math.degrees(sum_angle))
                break
        if with_stop:
            self.send_speed_cmd(0.0, 0.0)

    def wait(self, dt):  # TODO refactor to some common class
        while self.sim_time is None:
            self.update()
        start_sim_time = self.sim_time
        while self.sim_time - start_sim_time < dt:
            self.update()

    def lidar_drive_around(self, direction=None):
        # direction: only sign counts, positive will step around to the left, negative to the right
        print(self.sim_time, self.robot_name, "lidar_drive_around (speed: %.1f)" % (self.max_speed))
        timeout = timedelta(seconds=10) # add 4 sec for wheel setup

        # if substantially more close points to the left, go right and vice versa; if similar, go in one direction at random (avoids getting stuck)
        if direction is None:
            if self.scan_avg_distance_left < 0.8 * self.scan_avg_distance_right:
                direction = -1
            elif  self.scan_avg_distance_left * 0.8 > self.scan_avg_distance_right:
                direction = 1
            else:
                direction = 1 if self.rand.getrandbits(1) == 0 else -1

        start_pose = self.xyz
        start_time = self.sim_time
        while self.scan_distance_to_obstacle < 4000: # 4m
            self.publish("desired_movement", [GO_STRAIGHT, -math.copysign(5500, direction), -self.default_effort_level]) # 1000m radius is almost straight
            self.wait(timedelta(milliseconds=100))
            if timeout is not None and self.sim_time - start_time > timeout:
                print(self.sim_time, self.robot_name, "lidar_drive_around - timeout at %.1fm" % distance(start_pose, self.xyz))
                break

    def drive_around_rock(self, how_far, view_direction=None, timeout=None):
        # go around a rock with 'how_far' clearance, if how_far positive, it goes around to the left, negative means right
        # will keeping to look forward (i.e, movement is sideways)
        # view_direction - if present, first turn in place to point in that direction
        # TODO: use lidar to go around as much as needed
        print(self.sim_time, self.robot_name, "go_around_a_rock %.1f (speed: %.1f)" % (how_far, self.max_speed))
        if timeout is None:
            timeout = timedelta(seconds=4 + 2*abs(how_far) / self.max_speed) # add 4 sec for wheel setup
        if view_direction is not None:
            self.turn(normalizeAnglePIPI(view_direction - self.yaw), timeout=timedelta(seconds=10))

        start_pose = self.xyz
        start_time = self.sim_time
        while distance(start_pose, self.xyz) < abs(how_far):
            self.publish("desired_movement", [GO_STRAIGHT, 7500, -math.copysign(self.default_effort_level, how_far)]) # 1000m radius is almost straight
            self.wait(timedelta(milliseconds=100))
            if timeout is not None and self.sim_time - start_time > timeout:
                print(self.sim_time, self.robot_name, "go_around_a_rock - timeout at %.1fm" % distance(start_pose, self.xyz))
                break

        self.send_speed_cmd(0.0, 0.0) # TEST
        return # TEST TODO: this will end rock bypass routine right after side-step

        self.go_straight(math.copysign(abs(how_far) + 5, how_far)) # TODO: rock size estimate
        # TODO: maybe going back not needed, hand over to main routine, we presumably already cleared the obstacle
        start_pose = self.xyz
        start_time = self.sim_time
        while distance(start_pose, self.xyz) < abs(how_far):
            self.publish("desired_movement", [GO_STRAIGHT, -7500, -math.copysign(self.default_effort_level, how_far)]) # 1000m radius is almost straight
            self.wait(timedelta(milliseconds=100))
            if timeout is not None and self.sim_time - start_time > timeout:
                print(self.sim_time, self.robot_name, "go_around_a_rock - timeout at %.1fm" % distance(start_pose, self.xyz))
                break
        self.send_speed_cmd(0.0, 0.0)
        print(self.sim_time, self.robot_name, "go_around_a_rock ended at [%.1f,%.1f]" % (self.xyz[0], self.xyz[1]))


    def try_step_around(self):
        self.turn(math.radians(90), timeout=timedelta(seconds=10))

        # recovered enough at this point to switch to another driver (in case you see cubesat while doing the 3m drive or the final turn)
        self.bus.publish('driving_recovery', False)

        self.go_straight(5.0)
        self.turn(math.radians(-90), timeout=timedelta(seconds=10))

    def wait_for_init(self):
        print(self.robot_name, 'Wait for definition of last_position and yaw')
        while self.sim_time is None or self.last_position is None or self.yaw is None:
            try:
                self.update()
            except BusShutdownException:
                raise
            except:
                pass
        print(self.robot_name, 'done at', self.sim_time)

# vim: expandtab sw=4 ts=4
