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

CAMERA_ANGLE_DRIVING = 0.1
CAMERA_ANGLE_LOOKING = 0.5
CAMERA_ANGLE_HOMEBASE = 0.25 # look up while circling around homebase to avoid fake reflections from surrounding terrain
CUBESAT_MIN_EDGE_DISTANCE = 100

MAX_NR_OF_FARTHER_SCANS = 20
HOMEBASE_KEEP_DISTANCE = 3 # maintain this distance from home base while approaching and going around
HOMEBASE_RADIUS = 4 # radius of the homebase structure (i.e, estimated 8m diameter)

MAX_BASEMARKER_DISTANCE_HISTORY = 20 # do not act on a single lidar measurement, look at this many at a time and pick the min value
MAX_BASEMARKER_DISTANCE = 15

SPEED_ON = 10 # only +/0/- matters
TURN_ON = 10 # radius of circle when turning
GO_STRAIGHT = float("inf")

# DEBUG launch options
SKIP_CUBESAT_SUCCESS = False # skip cubesat, try to reach homebase directly
SKIP_HOMEBASE_SUCCESS = False # do not require successful homebase arrival report in order to look for alignment

ATTEMPT_DELAY = timedelta(seconds=20)

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

        self.basemarker_centered = False
        self.basemarker_left_history = []
        self.basemarker_right_history = []
        self.basemarker_whole_scan_history = []
        self.basemarker_radius = None
        self.centering = False
        self.going_around_count = 0

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
                            self.going_around_count += 1
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

        observed_values = [(23.5, 30.7), (27.5, 24.5), (28, 21.35), (29.5, 20.5), (41,18.3), (45,15.5), (51, 15.1), (58.5, 12), (62, 11.9)]

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

        self.objects_in_view[artifact_type] = {
            "expiration": self.sim_time + timedelta(milliseconds=200)
            }

        if self.last_attempt_timestamp is not None and self.sim_time - self.last_attempt_timestamp < ATTEMPT_DELAY:
            return


        center_x = data[1] + data[3] / 2
        center_y = data[2] + data[4] / 2
        bbox_size = (data[3] + data[4]) / 2 # calculate avegage in case of substantially non square matches
        img_x, img_y, img_w, img_h = data[1:5]
        nr_of_black = data[4]

#        print ("Artf: %s %d %d %d %d %d" % (artifact_type, img_x, img_y, img_w, img_h, nr_of_black))

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
                            self.camera_angle > 0.8 * CAMERA_ANGLE_LOOKING and
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
                            nonlocal data
                            if message.split()[0] == 'origin':
                                origin = [float(x) for x in message.split()[1:]]
                                self.xyz = origin[:3]
                                qx, qy, qz, qw = origin[3:]

                                self.publish('pose3d', [self.xyz, origin[3:]])

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
                                angle_x = math.atan( (CAMERA_WIDTH / 2 - (img_x + img_w/2) ) / float(CAMERA_FOCAL_LENGTH))
                                angle_y = math.atan( (CAMERA_HEIGHT / 2 - (img_y + img_h/2) ) / float(CAMERA_FOCAL_LENGTH))

                                distance = self.interpolate_distance((img_w + img_h) / 2)
                                ax = self.nasa_yaw + angle_x
                                ay = self.nasa_pitch + angle_y

                                if self.use_gimbal:
                                    # gimbal changes the actual angle dynamically so pitch needs to be offset
                                    ay += min(math.pi / 4.0, max(-math.pi / 8.0, self.camera_angle - self.nasa_pitch))
                                else:
                                    ay += self.camera_angle


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
                                        # time to start looking for homebase; TODO queue 360 look around as base is somewhere near
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
                    if center_x < (CAMERA_WIDTH/2 - 5): # if marker to the left
                        self.basemarker_centered = False
                    elif center_x > (CAMERA_WIDTH/2 + 5):
                        self.basemarker_centered = False
                    else:
                        print(self.sim_time, "app: basemarker centered")
                        self.basemarker_centered = True



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
            self.basemarker_centered = False
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
                    self.basemarker_centered = False
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

                def pol2cart(rho, phi):
                    x = rho * math.cos(phi)
                    y = rho * math.sin(phi)
                    return(x, y)

                x_l = []
                y_l = []
                for i in range(min_index, max_index):
                    x,y = pol2cart(data[i] / 1000.0, -1.29999995232 + (i - 40) * 0.0262626260519)
                    x_l.append(x)
                    y_l.append(y)

                homebase_cx, homebase_cy, homebase_radius = best_fit_circle(x_l, y_l)
                # print ("Center: [%f,%f], radius: %f" % (homebase_cx, homebase_cy, homebase_radius))

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

                if self.basemarker_centered and left_dist < 6 and abs(homebase_cy) < 0.1: # cos 20 = dist_r / dist _l is the max ratio in order to be at most 10 degrees off; also needs to be closer than 6m
                    self.publish("desired_movement", [0, -9000, 0])
                    self.object_reached('basemarker')
                    return

                # if re-centering rover towards the homebase, keep turning until very close to centered (as opposed to within the target range)
                if self.centering and abs(homebase_cy > 0.1):
                    return
                else:
                    self.centering = False

                # if seeing basemarker and homebase center is not straight ahead OR if looking past homebase in one of the directions, turn in place to adjust
                circle_direction = 1 if self.going_around_count % 2 == 0 else -1
                if (self.currently_following_object['object_type'] == 'basemarker' and homebase_cy < -0.2) or left_dist > 10:
                    self.centering = True
                    self.publish("desired_movement", [0, -9000, -SPEED_ON])
                elif (self.currently_following_object['object_type'] == 'basemarker' and homebase_cy > 0.2) or right_dist > 10:
                    self.centering = True
                    self.publish("desired_movement", [0, -9000, SPEED_ON])
                elif left_dist < 1.5 or right_dist < 1.5:
                    self.publish("desired_movement", [float("inf"), -9000, -SPEED_ON])
                elif left_dist > HOMEBASE_KEEP_DISTANCE + 1 or right_dist > HOMEBASE_KEEP_DISTANCE + 1:
                    self.publish("desired_movement", [float("inf"), -9000, SPEED_ON])
                elif homebase_cy < -1:
                    self.centering = True
                    self.publish("desired_movement", [0, -9000, -SPEED_ON])
                elif homebase_cy > 1:
                    self.centering = True
                    self.publish("desired_movement", [0, -9000, SPEED_ON])
                else:
                    # print ("driving radius: %f" % self.basemarker_radius)
                    # negative radius turns to the right
                    self.publish("desired_movement", [-(HOMEBASE_KEEP_DISTANCE + HOMEBASE_RADIUS), -9000, circle_direction * SPEED_ON])



    def run(self):

        try:
            self.wait_for_init()
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
            while self.sim_time - start_time < timedelta(minutes=40):
                additional_turn = 0
                last_walk_start = self.sim_time

                # TURN 360
                try:
                    self.virtual_bumper = VirtualBumper(timedelta(seconds=20), 0.1)
                    with LidarCollisionMonitor(self):
                        if self.current_driver is None and not self.brakes_on:
                            # start each straight stretch by looking around first
                            self.set_cam_angle(CAMERA_ANGLE_LOOKING)
                            self.turn(math.radians(360), timeout=timedelta(seconds=20))
                        else:
                            self.wait(timedelta(minutes=2)) # allow for self driving, then timeout
                except ChangeDriverException as e:
                    print(self.sim_time, "Turn interrupted by driver: %s" % e)
                    continue
                except (VirtualBumperException, LidarCollisionException):
                    self.inException = True
                    self.set_cam_angle(CAMERA_ANGLE_DRIVING)
                    print(self.sim_time, "Turn Virtual Bumper!")
                    # TODO: if detector takes over driving within initial turn, rover may be actually going straight at this moment
                    # also, it may be simple timeout, not a crash
                    self.virtual_bumper = None
                    # recovery from an exception while turning CCW is to turn CW somewhat
                    deg_angle = self.rand.randrange(-180, -90)
                    self.turn(math.radians(deg_angle), timeout=timedelta(seconds=10))
                    self.inException = False
                    self.bus.publish('driving_recovery', False)

                # GO STRAIGHT
                try:
                    self.virtual_bumper = VirtualBumper(timedelta(seconds=4), 0.1)
                    with LidarCollisionMonitor(self):
                        if self.current_driver is None and not self.brakes_on:
                            self.set_cam_angle(CAMERA_ANGLE_DRIVING)
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
