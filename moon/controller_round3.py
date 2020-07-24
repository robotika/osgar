"""
  Space Robotics Challenge 2
"""
import math
from statistics import median
from datetime import timedelta

import numpy as np

import moon.controller
from osgar.bus import BusShutdownException

from moon.controller import SpaceRoboticsChallenge, ChangeDriverException, VirtualBumperException, LidarCollisionException, LidarCollisionMonitor
from osgar.lib.virtual_bumper import VirtualBumper


CAMERA_FOCAL_LENGTH = 381
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480

CAMERA_BASELINE = 0.41 # distance between lenses

CAMERA_ANGLE_DRIVING = 0.1
CAMERA_ANGLE_LOOKING = 0.5
CAMERA_ANGLE_HOMEBASE = 0.25 # look up while circling around homebase to avoid fake reflections from surrounding terrain
CUBESAT_MIN_EDGE_DISTANCE = 130

MAX_NR_OF_FARTHER_SCANS = 20
HOMEBASE_KEEP_DISTANCE = 3 # maintain this distance from home base while approaching and going around
HOMEBASE_RADIUS = 4 # radius of the homebase structure (i.e, estimated 8m diameter)

MAX_BASEMARKER_DISTANCE_HISTORY = 20 # do not act on a single lidar measurement, look at this many at a time and pick the min value
MAX_BASEMARKER_DISTANCE = 15

SPEED_ON = 10 # only +/0/- matters
TURN_ON = 8 # radius of circle when turning
GO_STRAIGHT = float("inf")

# DEBUG launch options
SKIP_CUBESAT_SUCCESS = False # skip cubesat, try to reach homebase directly
SKIP_HOMEBASE_SUCCESS = False # do not require successful homebase arrival report in order to look for alignment

ATTEMPT_DELAY = timedelta(seconds=30)

def pol2cart(rho, phi):
    x = rho * math.cos(phi)
    y = rho * math.sin(phi)
    return(x, y)

def cart2pol(x, y):
    rho = np.sqrt(x**2 + y**2)
    phi = np.arctan2(y, x)
    return(rho, phi)

def min_dist(laser_data):
    if len(laser_data) > 0:
        # remove ultra near reflections and unlimited values == 0
        laser_data = [x if x > 10 else 20000 for x in laser_data] # NASA scanner goes up to 15m of valid measurement
        return min(laser_data)/1000.0
    return 0

def median_dist(laser_data):
    if len(laser_data) > 0:
        # remove ultra near reflections and unlimited values == 0
        laser_data = [x if x > 10 else 20000 for x in laser_data] # NASA scanner goes up to 15m of valid measurement
        return median(laser_data)/1000.0
    return 0

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

class SpaceRoboticsChallengeRound3(SpaceRoboticsChallenge):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register("desired_movement")

        self.cubesat_location = None
        self.homebase_arrival_success = False

        self.cubesat_reached = False
        self.cubesat_success = False

        self.basemarker_angle = None
        self.basemarker_left_history = []
        self.basemarker_right_history = []
        self.basemarker_whole_scan_history = []
        self.basemarker_radius = None
        self.centering = False
        self.going_around_count = 0
        self.full_360_scan = False
        self.full_360_objects = {}

        self.last_attempt_timestamp = None

        self.currently_following_object = {
            'object_type': None,
            'timestamp': None
            }

        self.object_timeouts = {
            'homebase': timedelta(seconds=5), # tracking homebase based on visual
            'cubesat': timedelta(seconds=5),
            'basemarker': timedelta(milliseconds=200)
            }


        self.last_artefact_time = None
        self.last_tracked_artefact = None

        self.objects_to_follow = []
        self.objects_in_view = {}


    def follow_object(self, data):
        self.objects_to_follow = data
        self.last_attempt_timestamp = None
        print (self.sim_time, "Starting to look for " + ','.join(data))

    def on_driving_control(self, data):
        super().on_driving_control(data)

        if data is None:
            self.set_cam_angle(CAMERA_ANGLE_DRIVING)
            print("Driving returned to main")
        else:
            print("Current driver: %s" % self.current_driver)
            if self.current_driver == "cubesat":
                self.set_cam_angle(CAMERA_ANGLE_LOOKING)
            elif self.current_driver == "homebase":
                self.set_cam_angle(CAMERA_ANGLE_DRIVING)
            else: # basemarker
                self.set_cam_angle(CAMERA_ANGLE_HOMEBASE)
        if not self.inException: # do not interrupt driving if processing an exception
            raise ChangeDriverException(data)

    def object_reached(self, object_type):
        self.currently_following_object['object_type'] = None
        self.currently_following_object['timestamp'] = None

        self.objects_to_follow.remove(object_type)

        if object_type == "homebase": # upon handover, robot should be moving straight
            if self.cubesat_success:
                if not self.homebase_arrival_success:

                    def process_arrival(response):
                        print(self.sim_time, "app: Homebase response: %s" % response)

                        if response == 'ok' or SKIP_HOMEBASE_SUCCESS:
                            self.set_cam_angle(CAMERA_ANGLE_HOMEBASE)
                            self.current_driver = "basemarker"
                            self.homebase_arrival_success = True
                            self.follow_object(['basemarker'])

                        else:
                            # homebase arrival not accepted, keep trying after a delay
                            self.follow_object(['homebase'])
                            self.last_attempt_timestamp = self.sim_time
                            self.on_driving_control(None) # do this last as it raises exception

                    self.send_request('artf homebase', process_arrival)

                else:
                    # homebase found (again), does not need reporting, just start basemarker search
                    self.set_cam_angle(CAMERA_ANGLE_HOMEBASE)
                    self.current_driver = "basemarker"
                    self.going_around_count += 1
                    self.follow_object(['basemarker'])

            else:
                print(self.sim_time, "app: Reached reportable home base destination, need to find cubesat first though")
                self.on_driving_control(None) # do this last as it raises exception

        elif object_type == 'basemarker':
            print (self.sim_time, "app: Reporting alignment to server")

            def process_alignment(response):
                print(self.sim_time, "app: Alignment response: %s" % response)
                if response == 'ok':
                    # all done, exiting
                    exit
                else:
                    self.going_around_count += 1
                    self.follow_object(['basemarker'])

            self.send_request('artf homebase_alignment', process_alignment)


    def interpolate_distance(self, pixels):
        # linearly interpolate in between measured values (pixels, distance)
        # line from 2 points: https://www.desmos.com/calculator/md6buy4efz
        # plot 2D points: https://www.desmos.com/calculator/mhq4hsncnh
        # plot 3D points: https://technology.cpm.org/general/3dgraph/

        observed_values = [(294, 33.5), (370, 28.9), (396, 27.9), (499, 25.4), (565, 26), (589, 25.8), (661, 25.4), (958, 20), (1258, 17.5), (1635, 15.35), (2103, 13.56), (4594, 9.7)]

        t1 = None

        if pixels < observed_values[0][0]:
            x2 = observed_values[1][0]
            y2 = observed_values[1][1]
            x1 = observed_values[0][0]
            y1 = observed_values[0][1]
            m = (y2 - y1) / (x2 - x1)
            return m * (pixels - x1) + y1

        else:
            for t2 in observed_values:
                if pixels < t2[0]:
                    x2 = t2[0]
                    y2 = t2[1]
                    x1 = t1[0]
                    y1 = t1[1]
                    m = (y2 - y1) / (x2 - x1)
                    return m * (pixels - x1) + y1
                else:
                    t1 = t2

        i = len(observed_values) - 2
        x2 = observed_values[i+1][0]
        y2 = observed_values[i+1][1]
        x1 = observed_values[i][0]
        y1 = observed_values[i][1]
        m = (y2 - y1) / (x2 - x1)
        return m * (pixels - x1) + y1

    # used to follow objects (cubesat, processing plant, other robots, etc)
    def on_artf(self, data):
        # vol_type, x, y, w, h
        # coordinates are pixels of bounding box
        artifact_type = data[0]

        if self.sim_time is None:
            return

        self.objects_in_view[artifact_type] = {
            "expiration": self.sim_time + timedelta(milliseconds=200)
            }

        if self.last_attempt_timestamp is not None and self.sim_time - self.last_attempt_timestamp < ATTEMPT_DELAY:
            return


        center_x = data[1] + data[3] / 2
        center_y = data[2] + data[4] / 2
        bbox_size = (data[3] + data[4]) / 2 # calculate avegage in case of substantially non square matches
        img_x, img_y, img_w, img_h = data[1:5]
        nr_of_nonblack = data[5]

        if self.full_360_scan:
            if artifact_type not in self.full_360_objects.keys():
                self.full_360_objects[artifact_type] = []
            self.full_360_objects[artifact_type].append(self.yaw)
            return

#        print ("Artf: %s %d %d %d %d %d" % (artifact_type, img_x, img_y, img_w, img_h, nr_of_nonblack))

        # TODO if detection during turning on the spot, instead of driving straight steering a little, turn back to the direction where the detection happened first

        if (
                not self.in_driving_recovery and
                self.objects_to_follow and
                artifact_type in self.objects_to_follow
        ): # if in exception, let the exception handling take its course
            if self.currently_following_object['object_type'] is None:
                self.currently_following_object['object_type'] = artifact_type
                self.currently_following_object['timestamp'] = self.sim_time
                print (self.sim_time, "Starting to track %s" % artifact_type)
                self.on_driving_control(artifact_type)
            else:
                for looking_for in self.objects_to_follow:
                    if self.currently_following_object['object_type'] == looking_for: # detected artefact is top priority and it is the one being followed already, continue what you were doing
                        break
                    elif looking_for == artifact_type: # we are looking for this artifact but it's not the one currently being followed, switch
                        print (self.sim_time, "Switching to tracking %s" % artifact_type)
                        self.currently_following_object['object_type'] = artifact_type
                        self.on_driving_control(artifact_type)

            if self.currently_following_object['object_type'] == artifact_type:
                self.currently_following_object['timestamp'] = self.sim_time

                if self.currently_following_object['object_type'] == 'cubesat':
                    #            print("Cubesat reported at %d %d %d %d" % (data[1], data[2], data[3], data[4]))
                    # virtual bumper still applies while this block has control. When triggered, driving will go to recovery and main will take over driving

                    # when cubesat disappears, we need to reset the steering to going straight
                    # NOTE: light does not shine in corners of viewport, need to report sooner or turn first
                    # box is big enough and close to the top edge and camera is pointing up and far enough from x edges, report
                    if (
                            bbox_size > 25 and
                            img_y < 40 and
                            abs(self.sensor_joint_position - (CAMERA_ANGLE_LOOKING + self.pitch)) < 0.1 and # camera is close to 'looking up' position
                            CUBESAT_MIN_EDGE_DISTANCE < img_x < (CAMERA_WIDTH - CUBESAT_MIN_EDGE_DISTANCE - bbox_size)
                    ):
                        # box 25 pixels represents distance about 27m which is as close as we can possibly get for cubesats with high altitude
                        # object in center (x axis) and close enough (bbox size)
                        # stop and report angle and distance from robot
                        # robot moves a little after detection so the angles do not correspond with the true pose we will receive
                        # TODO: if found during side sweep, robot will turn some between last frame and true pose messing up the angle
                        self.set_brakes(True)
                        self.publish("desired_movement", [0, 0, 0])
                        print(self.sim_time, "app: cubesat final frame x=%d y=%d w=%d h=%d" % (data[1], data[2], data[3], data[4]))

                        if 'homebase' in self.objects_to_follow:
                            self.objects_to_follow.remove('homebase') # do not immediately follow homebase if it was secondary to give main a chance to report cubesat

                        def process_origin(message):
                            self.register_origin(message)
                            nonlocal data
                            if message.split()[0] == 'origin':
                                origin = [float(x) for x in message.split()[1:]]
                                self.xyz = origin[:3]
                                qx, qy, qz, qw = origin[3:]

                                print(self.sim_time, "Origin received, internal position updated")
                                # robot should be stopped right now (using brakes once available)
                                # lift camera to max, object should be (back) in view
                                # trigger recognition, get bounding box and calculate fresh angles

                                # TODO: this separate storage of reported numbers is temporary, need OSGAR to accept true values and update its own data
                                self.nasa_xyz = self.xyz
                                sinr_cosp = 2 * (qw * qx + qy * qz);
                                cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
                                self.nasa_roll = math.atan2(sinr_cosp, cosr_cosp);

                                sinp = 2 * (qw * qy - qz * qx);
                                if abs(sinp) >= 1:
                                    self.nasa_pitch = math.copysign(math.pi / 2, sinp);
                                else:
                                    self.nasa_pitch = math.asin(sinp);

                                self.nasa_pitch = - self.nasa_pitch # for subsequent calculations, up is positive and down is negative

                                siny_cosp = 2 * (qw * qz + qx * qy);
                                cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
                                self.nasa_yaw = math.atan2(siny_cosp, cosy_cosp);
                                print (self.sim_time, "app: True pose received: xyz=[%f,%f,%f], roll=%f, pitch=%f, yaw=%f" % (origin[0],origin[1],origin[2],self.nasa_roll, self.nasa_pitch, self.nasa_yaw))

                                print(self.sim_time, "app: Final frame x=%d y=%d w=%d h=%d, nonblack=%d" % (data[1], data[2], data[3], data[4], data[5]))
                                screen_x = (CAMERA_WIDTH / 2 - (img_x + img_w/2) )
                                screen_y = (CAMERA_HEIGHT / 2 - (img_y + img_h/2) )

                                (rho, phi) = cart2pol(screen_x, screen_y)
                                phi += self.nasa_roll
                                (screen_x, screen_y) = pol2cart(rho, phi)

                                angle_x = math.atan( screen_x / float(CAMERA_FOCAL_LENGTH))
                                angle_y = math.atan( screen_y / float(CAMERA_FOCAL_LENGTH))

                                distance = self.interpolate_distance(nr_of_nonblack)
                                ax = self.nasa_yaw + angle_x
                                ay = self.nasa_pitch + angle_y

                                print (self.sim_time, "app: Camera angle adjustment: %f" % self.sensor_joint_position)
                                ay += self.sensor_joint_position

                                x, y, z = self.nasa_xyz
                                print("Using pose: xyz=[%f %f %f] orientation=[%f %f %f]" % (x, y, z, self.nasa_roll, self.nasa_pitch, self.nasa_yaw))
                                print("In combination with view angle %f %f and distance %f" % (ax, ay, distance))
                                ox = math.cos(ax) * math.cos(ay) * distance
                                oy = math.sin(ax) * math.cos(ay) * distance
                                oz = math.sin(ay) * distance
                                self.cubesat_location = (x+ox, y+oy, z+oz)
                                print (self.sim_time, "app: Object offset calculated at: [%f %f %f]" % (ox, oy, oz))
                                print (self.sim_time, "app: Reporting estimated object location at: [%f,%f,%f]" % (x+ox, y+oy, z+oz))

                                def process_apriori_object(response):
                                    if response == 'ok':
                                        print("app: Apriori object reported correctly")
                                        self.cubesat_success = True
                                        self.object_reached("cubesat")
                                        self.follow_object(['homebase'])
                                    else:
                                        print("app: Estimated object location incorrect, wait before continuing task; response: %s" % str(response))
                                        self.last_attempt_timestamp = self.sim_time

                                s = '%s %.2f %.2f %.2f\n' % (artifact_type, x+ox, y+oy, z+oz)
                                self.send_request('artf ' + s, process_apriori_object)
                            else:
                                print(self.sim_time, "Origin request failed") # TODO: in future, we should try to find the cubesat again based on accurate position tracking
                                self.last_attempt_timestamp = self.sim_time

                            # TODO: object was reached but not necessarily successfully reported;
                            # for now, just return driving to main which will either drive randomly with no additional purpose if homebase was previously found or will look for homebase
                            self.set_brakes(False)
                            self.current_driver = None
                            self.on_driving_control(None) # do this last as it raises exception

                        self.send_request('request_origin', process_origin)

                    elif center_x < 200: # if cubesat near left edge, turn left
                        if img_y > 20: # if far enough from top, go straight too, otherwise turn in place
                            self.publish("desired_movement", [TURN_ON, 0, SPEED_ON])
                        else:
                            self.publish("desired_movement", [0, 0, SPEED_ON])
                    elif center_x > 440:
                        if img_y > 20: # if far enough from top, go straight too, otherwise turn in place
                            self.publish("desired_movement", [-TURN_ON, 0, SPEED_ON])
                        else:
                            self.publish("desired_movement", [0, 0, -SPEED_ON])
                    else:
                        # bbox is ahead but too small or position not near the edge, continue straight
                        self.publish("desired_movement", [GO_STRAIGHT, 0, SPEED_ON])


                elif self.currently_following_object['object_type'] == 'homebase':
                    if center_x < (CAMERA_WIDTH/2 - 20): # if homebase to the left, steer left
                        self.publish("desired_movement", [TURN_ON, 0, SPEED_ON])
                    elif center_x > (CAMERA_WIDTH/2 + 20):
                        self.publish("desired_movement", [-TURN_ON, 0, SPEED_ON])
                    else: # if centered, keep going straight
                        self.publish("desired_movement", [GO_STRAIGHT, 0, SPEED_ON])

                elif self.currently_following_object['object_type'] == 'basemarker':
                    self.basemarker_angle = math.atan( (CAMERA_WIDTH / 2 - center_x ) / float(CAMERA_FOCAL_LENGTH))



    def on_scan(self, data):
        assert len(data) == 180
        super().on_scan(data)

        delete_in_view = [artf for artf in self.objects_in_view if self.objects_in_view[artf]['expiration'] < self.sim_time]
        for artf in delete_in_view:
            del self.objects_in_view[artf]

        if self.last_attempt_timestamp is not None and self.sim_time - self.last_attempt_timestamp < ATTEMPT_DELAY:
            return

        # if was following an artefact but it disappeared, just go straight until another driver takes over
        if (
                self.currently_following_object['timestamp'] is not None and
                self.currently_following_object['object_type'] is not None and
                self.sim_time - self.currently_following_object['timestamp'] > self.object_timeouts[self.currently_following_object['object_type']]

        ):
            self.publish("desired_movement", [GO_STRAIGHT, 0, SPEED_ON])
            print (self.sim_time, "No longer tracking %s" % self.currently_following_object['object_type'])
            self.currently_following_object['timestamp'] = None
            self.currently_following_object['object_type'] = None
            self.basemarker_angle = None
            if self.current_driver != "basemarker": # do not change drivers when basemarker gets out of view because going around will bring it again
                self.on_driving_control(None) # do this last as it raises exception


        # NASA sends 100 samples over 2.6rad (~150 degrees) (each increment is 0.0262626260519rad (~15 degrees))
        # OSGAR pads in in front and in back to 180 samples, first and last 40 are zeros

# TODO: if too far from anything, revert to looking for homebase
        if not self.in_driving_recovery:

            midindex = len(data) // 2
            # 10 degrees left and right is 6-7 samples before and after the array midpoint
            straight_ahead_dist = min_dist(data[midindex-15:midindex+15])

            if self.currently_following_object['object_type'] == 'homebase':
#                print (self.sim_time, "controller_round3: homebase current distance: %f" % (straight_ahead_dist))

                if straight_ahead_dist < HOMEBASE_KEEP_DISTANCE:
                    self.publish("desired_movement", [0, 0, 0])

                    print ("app: final homebase distance %f: " % straight_ahead_dist)
                    self.object_reached('homebase')

            if 'basemarker' in self.objects_to_follow:

                if 'homebase' not in self.objects_in_view.keys():
                    # lost contact with homebase, try approach again
                    print (self.sim_time, "app: No longer going around homebase")
                    self.currently_following_object['timestamp'] = None
                    self.currently_following_object['object_type'] = None
                    self.basemarker_angle = None
                    self.basemarker_right_history = self.basemarker_left_history = []
                    self.follow_object(['homebase'])
                    self.on_driving_control(None) # do this last as it possibly raises exception
                    return

                # find min_index where distance < 5 and max_index where distance < 5
                # find circle passing through these points (defined in polar coordinates as [index*6 degrees, distance]
                min_index = max_index = None
                for i in range(0,180):
                    if 10 < data[i] < 5000:
                        min_index = i
                        break
                for i in range(180, 0, -1):
                    if 10 < data[i-1] < 5000:
                        max_index = i
                        break

                if min_index is None or max_index is None or max_index - min_index < 3:
                    # if in basemarker mode, looking at homebase but lidar shows no hits, it's a noisy lidar scan, ignore
                    return

                x_l = []
                y_l = []
                for i in range(min_index, max_index):
                    x,y = pol2cart(data[i] / 1000.0, -1.29999995232 + (i - 40) * 0.0262626260519)
                    x_l.append(x)
                    y_l.append(y)

                homebase_cx, homebase_cy, homebase_radius = best_fit_circle(x_l, y_l)
                # print ("Center: [%f,%f], radius: %f" % (homebase_cx, homebase_cy, homebase_radius))

                # since we are looking through left camera, cy will be calculated relative to the camera
                # need it relative to the center of the robot
                homebase_cy -= CAMERA_BASELINE / 2

                right_dist = median_dist(data[midindex-8:midindex-6])
                left_dist = median_dist(data[midindex+6:midindex+8])
                self.basemarker_left_history.append(left_dist)
                self.basemarker_right_history.append(right_dist)
                self.basemarker_whole_scan_history.append(min_dist(data))
                if len(self.basemarker_right_history) > MAX_BASEMARKER_DISTANCE_HISTORY:
                    self.basemarker_right_history.pop(0)
                if len(self.basemarker_left_history) > MAX_BASEMARKER_DISTANCE_HISTORY:
                    self.basemarker_left_history.pop(0)
                if len(self.basemarker_whole_scan_history) > MAX_BASEMARKER_DISTANCE_HISTORY:
                    self.basemarker_whole_scan_history.pop(0)
                left_dist = min(self.basemarker_left_history)
                right_dist = min(self.basemarker_right_history)

                # print (self.sim_time, "app: Min dist front: %f, dist left=%f, right=%f" % (straight_ahead_dist, left_dist, right_dist))

                if self.basemarker_angle is not None and abs(self.basemarker_angle) < 0.06 and left_dist < 6 and abs(homebase_cy) < 0.1: # cos 20 = dist_r / dist _l is the max ratio in order to be at most 10 degrees off; also needs to be closer than 6m
                    self.publish("desired_movement", [0, -9000, 0])
                    self.object_reached('basemarker')
                    return

                # if re-centering rover towards the homebase, keep turning until very close to centered (as opposed to within the target range)
                if self.centering and abs(homebase_cy > 0.1):
                    return
                else:
                    self.centering = False

                # if seeing basemarker and homebase center is not straight ahead OR if looking past homebase in one of the directions, turn in place to adjust
                # -1 means going right, 1 going left
                circle_direction = 1 if self.going_around_count % 2 == 0 else -1
                if (self.currently_following_object['object_type'] == 'basemarker' and homebase_cy < -0.2) or left_dist > 10:
                    self.centering = True
                    self.publish("desired_movement", [0, -9000, -SPEED_ON])
                elif (self.currently_following_object['object_type'] == 'basemarker' and homebase_cy > 0.2) or right_dist > 10:
                    self.centering = True
                    self.publish("desired_movement", [0, -9000, SPEED_ON])
                elif left_dist < 1.5 or right_dist < 1.5:
                    self.publish("desired_movement", [GO_STRAIGHT, -9000, -SPEED_ON])
                elif left_dist > HOMEBASE_KEEP_DISTANCE + 1 or right_dist > HOMEBASE_KEEP_DISTANCE + 1:
                    self.publish("desired_movement", [GO_STRAIGHT, -9000, SPEED_ON])
                elif homebase_cy < -1:
                    self.centering = True
                    self.publish("desired_movement", [0, -9000, -SPEED_ON])
                elif homebase_cy > 1:
                    self.centering = True
                    self.publish("desired_movement", [0, -9000, SPEED_ON])
                else:
                    # print ("driving radius: %f" % self.basemarker_radius)
                    # negative radius turns to the right
                    if self.currently_following_object['object_type'] == 'basemarker' and self.basemarker_angle is not None:
                        (rho, phi) = cart2pol(homebase_cx, homebase_cy)
                        circle_direction = -1 if self.basemarker_angle < phi else 1
                    self.publish("desired_movement", [-(HOMEBASE_KEEP_DISTANCE + HOMEBASE_RADIUS), -9000, circle_direction * SPEED_ON])



    def run(self):

        try:
            self.wait_for_init()
            self.set_light_intensity("1.0")
            self.set_brakes(False)
            # some random manual starting moves to choose from
#            self.go_straight(-0.1, timeout=timedelta(seconds=20))
#            self.go_straight(-2, timeout=timedelta(seconds=20))
#            self.turn(math.radians(45), timeout=timedelta(seconds=20))
#            self.set_cam_angle(CAMERA_ANGLE_HOMEBASE)



            if SKIP_CUBESAT_SUCCESS:
                # skip cubesat
                self.cubesat_success = True
                self.follow_object(['homebase'])
            else:
                # regular launch
                self.follow_object(['cubesat', 'homebase'])

#            self.homebase_arrival_success = True
#            self.cubesat_success = True
#            self.follow_object(['homebase'])
#            self.follow_object(['basemarker'])
#            self.current_driver = 'basemarker'

            #            self.publish("desired_movement", [10, 9000, 10])
#            self.wait(timedelta(seconds=10))

            last_walk_start = 0.0
            start_time = self.sim_time
            while self.sim_time - start_time < timedelta(minutes=45):
                additional_turn = 0
                last_walk_start = self.sim_time

                # TURN 360
                # TODO:
                # if looking for multiple objects, sweep multiple times, each looking only for one object (objects listed in order of priority)
                # this way we won't start following homebase when cubesat was also visible, but later in the sweep
                # alternatively, sweep once without immediately reacting to matches but note match angles
                # then turn back to the direction of the top priority match

                if self.current_driver is None and not self.brakes_on:
                    if len(self.objects_to_follow) > 1:
                        self.full_360_scan = True
                    try:
                        self.virtual_bumper = VirtualBumper(timedelta(seconds=20), 0.1)
                        with LidarCollisionMonitor(self):
                            # start each straight stretch by looking around first
                            # if cubesat already found, we are looking for homebase, no need to lift camera as much
                            self.set_cam_angle(CAMERA_ANGLE_DRIVING if self.cubesat_success else CAMERA_ANGLE_LOOKING)
                            self.turn(math.radians(360), timeout=timedelta(seconds=20))
                    except ChangeDriverException as e:
                        print(self.sim_time, "Turn interrupted by driver: %s" % e)
                        self.full_360_scan = False
                        self.full_360_objects = {}
                        continue
                        # proceed to straight line drive where we wait; straight line exception handling is better applicable for follow-object drive
                    except (VirtualBumperException, LidarCollisionException)  as e:
                        self.inException = True
                        self.set_cam_angle(CAMERA_ANGLE_DRIVING)
                        print(self.sim_time, "Turn 360 degrees Virtual Bumper!")
                        self.virtual_bumper = None
                        # recovery from an exception while turning CCW is to turn CW somewhat
                        deg_angle = self.rand.randrange(-180, -90)
                        self.turn(math.radians(deg_angle), timeout=timedelta(seconds=10))
                        self.inException = False
                        self.bus.publish('driving_recovery', False)
                    if len(self.objects_to_follow) > 1:
                        print(self.sim_time, "app: 360deg scan: " + str([[a, median(self.full_360_objects[a])] for a in self.full_360_objects.keys()]))
                        for o in self.objects_to_follow:
                            if o in self.full_360_objects.keys():
                                try:
                                    self.virtual_bumper = VirtualBumper(timedelta(seconds=20), 0.1)
                                    with LidarCollisionMonitor(self):
                                        self.turn(median(self.full_360_objects[o]) - self.yaw, timeout=timedelta(seconds=20))
                                except:
                                    # in case it gets stuck turning, just hand over driving to main without completing the desired turn
                                    pass
                                break
                        self.full_360_scan = False
                        self.full_360_objects = {}

                else:
                    try:
                        self.virtual_bumper = VirtualBumper(timedelta(seconds=20), 0.1)
                        with LidarCollisionMonitor(self):
                            self.wait(timedelta(minutes=2)) # allow for self driving, then timeout
                    except ChangeDriverException as e:
                        # if driver lost, start by turning 360; if driver changed, wait here again
                        continue
                    except (VirtualBumperException, LidarCollisionException) as e:
                        print(self.sim_time, "Follow-object Virtual Bumper")
                        self.inException = True
                        print(self.sim_time, repr(e))
                        last_walk_end = self.sim_time
                        self.virtual_bumper = None
                        self.set_cam_angle(CAMERA_ANGLE_DRIVING)
                        self.go_straight(-2.0, timeout=timedelta(seconds=10)) # this should be reasonably safe, we just came from there
                        self.try_step_around()
                        self.inException = False
                        self.bus.publish('driving_recovery', False)

                # GO STRAIGHT
                try:
                    self.virtual_bumper = VirtualBumper(timedelta(seconds=4), 0.1)
                    with LidarCollisionMonitor(self):
                        if self.current_driver is None and not self.brakes_on:
                            self.set_cam_angle(CAMERA_ANGLE_DRIVING)
                            self.go_straight(30.0, timeout=timedelta(minutes=2))
                        else:
                            self.wait(timedelta(minutes=2)) # allow for self driving, then timeout
                    self.update()
                except ChangeDriverException as e:
                    continue
                except (VirtualBumperException, LidarCollisionException) as e:
                    print(self.sim_time, "Go Straight or Follow-object Virtual Bumper")
                    self.inException = True
                    # TODO: crashes if an exception (e.g., excess pitch) occurs while handling an exception (e.g., virtual/lidar bump)
                    print(self.sim_time, repr(e))
                    last_walk_end = self.sim_time
                    self.virtual_bumper = None
                    self.set_cam_angle(CAMERA_ANGLE_DRIVING)
                    self.go_straight(-2.0, timeout=timedelta(seconds=10)) # this should be reasonably safe, we just came from there
                    if last_walk_end - last_walk_start > timedelta(seconds=20): # if we went more than 20 secs, try to continue a step to the left
                        # TODO: this is not necessarily safe, need to protect against pitch, etc again
                        self.try_step_around()
                    else:
                        self.bus.publish('driving_recovery', False)

                    self.inException = False

                    print ("Time elapsed since start of previous leg: %d sec" % (last_walk_end.total_seconds()-last_walk_start.total_seconds()))
                    if last_walk_end - last_walk_start > timedelta(seconds=40):
                        # if last step only ran short time before bumper (this includes bumper timeout time), time for a large random turn
                        # if it ran long time, maybe worth trying going in the same direction
                        continue
                    additional_turn = 30

                    # next random walk direction should be between 30 and 150 degrees
                    # (no need to go straight back or keep going forward)
                    # if part of virtual bumper handling, add 30 degrees to avoid the obstacle more forcefully


                # TURN RANDOMLY
                deg_angle = self.rand.randrange(30 + additional_turn, 150 + additional_turn)
                deg_sign = self.rand.randint(0,1)
                if deg_sign:
                    deg_angle = -deg_angle
                try:
                    self.virtual_bumper = VirtualBumper(timedelta(seconds=20), 0.1)
                    with LidarCollisionMonitor(self):
                        self.turn(math.radians(deg_angle), timeout=timedelta(seconds=30))
                except ChangeDriverException as e:
                    continue
                except (VirtualBumperException, LidarCollisionException):
                    self.inException = True
                    self.set_cam_angle(CAMERA_ANGLE_DRIVING)
                    print(self.sim_time, "Random Turn Virtual Bumper!")
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
