"""
  Space Robotics Challenge 2
"""
import math
from statistics import median
from datetime import timedelta

import moon.controller
from osgar.bus import BusShutdownException

from moon.controller import SpaceRoboticsChallenge, ChangeDriverException, VirtualBumperException, LidarCollisionException, LidarCollisionMonitor
from osgar.lib.virtual_bumper import VirtualBumper


CAMERA_FOCAL_LENGTH = 381
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480

CAMERA_ANGLE_DRIVING = 0.1
CAMERA_ANGLE_LOOKING = 0.5
CAMERA_ANGLE_CUBESAT = 0.78
CAMERA_ANGLE_HOMEBASE = 0.0

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

        
        self.homebase_final_approach_distance = float("inf")
        self.homebase_increase_count = 0 # if distance increases X times in a row, we are truly farther and need to restart

        self.last_attempt_timestamp = None
        
        self.currently_following_object = {
            'object_type': None,
            'timestamp': None
            }

        self.trackable_objects = [
            {
                "homebase": {
                    "timeout": timedelta(seconds=5),
                    "reached": False
                }
            },
            {
                "cubesat": {
                    "timeout": timedelta(seconds=5),
                    "reached": False
                }
            },
            {
                "basemarker": {
                    "timeout": timedelta(milliseconds=200),
                    "reached": False
                }
            }
        ]
        
        self.object_timeouts = {
            'homebase': timedelta(seconds=5), # tracking homebase based on visual
            'cubesat': timedelta(seconds=5),
            'basemarker': timedelta(milliseconds=200)
            }

        
        self.last_artefact_time = None
        self.last_tracked_artefact = None
        
        self.in_driving_recovery = False
        self.objects_to_follow = []


    def on_driving_recovery(self, data):
        self.in_driving_recovery = data
        print (self.time, "Driving recovery changed to: %r" % data)

    def follow_object(self, data):
        self.objects_to_follow = data
        self.last_attempt_timestamp = None
        print (self.time, "Starting to look for " + ','.join(data))

    def on_driving_control(self, timestamp, data):
        super().on_driving_control(timestamp, data)
        
        if data is None:
            self.set_cam_angle(CAMERA_ANGLE_DRIVING)
            print("Driving returned to main")
        else:
            print("Current driver: %s" % self.current_driver)
            if self.current_driver == "cubesat":
                self.set_cam_angle(CAMERA_ANGLE_LOOKING)
            elif self.current_driver == "homebase":
                self.set_cam_angle(CAMERA_ANGLE_DRIVING)
            elif self.current_driver == "homebase-final":
                self.set_cam_angle(CAMERA_ANGLE_DRIVING)
            else:
                self.set_cam_angle(CAMERA_ANGLE_DRIVING)
        if not self.inException: # do not interrupt driving if processing an exception
            raise ChangeDriverException(data)

    def object_reached(self, object_type):
        self.currently_following_object['object_type'] = None
        self.currently_following_object['timestamp'] = None

        self.objects_to_follow.remove(object_type)
            
        if object_type == "homebase": # upon handover, robot should be moving straight
            if self.cubesat_success:
                if not self.homebase_arrival_success:
                    response = self.send_request('artf homebase\n').decode("ascii") 
                    print(self.time, "app: Homebase response: %s" % response)

                    if response == 'ok' or SKIP_HOMEBASE_SUCCESS:
                        self.set_cam_angle(CAMERA_ANGLE_HOMEBASE)
                        self.current_driver = "basemarker"
                        self.homebase_arrival_success = True
                        self.follow_object(['basemarker'])

                    else:
                        # homebase arrival not accepted, keep trying after a delay
                        self.last_attempt_timestamp = self.time
                        self.on_driving_control(self.time, None) # do this last as it raises exception
                        
                else:
                    # homebase found (again), does not need reporting, just start basemarker search
                    self.set_cam_angle(CAMERA_ANGLE_HOMEBASE)
                    self.current_driver = "basemarker"
                    self.follow_object(['basemarker'])
                    
            else:
                print(self.time, "app: Reached reportable home base destination, need to find cubesat first though")
                self.on_driving_control(self.time, None) # do this last as it raises exception

        elif object_type == 'basemarker':
            print (self.time, "app: Reporting alignment to server")
            response = self.send_request('artf homebase_alignment\n').decode("ascii") 
            print(self.time, "app: Alignment response: %s" % response)
            if response == 'ok':
                # all done, exiting
                exit
            else:
                # do nothing, ie keep going around and try to match the view
                pass


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
    def on_artf(self, timestamp, data):


        if self.last_attempt_timestamp is not None and self.time - self.last_attempt_timestamp < ATTEMPT_DELAY:
            return

        # vol_type, x, y, w, h
        # coordinates are pixels of bounding box
        artifact_type = data[0]

        center_x = data[1] + data[3] / 2
        center_y = data[2] + data[4] / 2
        bbox_size = (data[3] + data[4]) / 2 # calculate avegage in case of substantially non square matches
        img_x, img_y, img_w, img_h = data[1:5]
        nr_of_black = data[4]

        # TODO if detection during turning on the spot, instead of driving straight steering a little, turn back to the direction where the detection happened first
        
        if (
                not self.in_driving_recovery and
                self.objects_to_follow and
                artifact_type in self.objects_to_follow
        ): # if in exception, let the exception handling take its course
            if self.currently_following_object['object_type'] is None:
                self.currently_following_object['object_type'] = artifact_type
                self.currently_following_object['timestamp'] = self.time
                print (self.time, "Starting to track %s" % artifact_type)
                self.on_driving_control(timestamp, artifact_type)
            else:
                for looking_for in self.objects_to_follow:
                    if self.currently_following_object['object_type'] == looking_for: # detected artefact is top priority and it is the one being followed already, continue what you were doing
                        break 
                    elif looking_for == artifact_type: # we are looking for this artifact but it's not the one currently being followed, switch
                        print (self.time, "Switching to tracking %s" % artifact_type)
                        self.currently_following_object['object_type'] = artifact_type
                        self.on_driving_control(timestamp, artifact_type)

            if self.currently_following_object['object_type'] == artifact_type:
                self.currently_following_object['timestamp'] = self.time

                if self.currently_following_object['object_type'] == 'cubesat':
                    #            print("Cubesat reported at %d %d %d %d" % (data[1], data[2], data[3], data[4]))
                    # virtual bumper still applies while this block has control. When triggered, driving will go to recovery and main will take over driving

                    # when cubesat disappears, we need to reset the steering to going straight
                    # NOTE: light does not shine in corners of viewport, need to report sooner or turn first
                    if bbox_size > 25 and img_y < 40: # box is big enough to report on and close to the edge, report
                         # box 25 pixels represents distance about 27m which is as close as we can possibly get for cubesats with high altitude 
                        # object in center (x axis) and close enough (bbox size)
                        # stop and report angle and distance from robot
                        # robot moves a little after detection so the angles do not correspond with the true pose we will receive
                        # TODO: if found during side sweep, robot will turn some between last frame and true pose messing up the angle
                        self.set_brakes(True)
                        self.publish("desired_movement", [0, 0, 0])
                        print(self.time, "app: cubesat final frame x=%d y=%d w=%d h=%d" % (data[1], data[2], data[3], data[4]))

                        if 'homebase' in self.objects_to_follow:
                            self.objects_to_follow.remove('homebase') # do not immediately follow homebase if it was secondary to give main a chance to report cubesat

                        message = self.send_request('request_origin') # response to this is required, if none, rover will be stopped forever
                        if message.split()[0] == b'origin':
                            origin = [float(x) for x in message.split()[1:]]
                            self.xyz = origin[:3]
                            qx, qy, qz, qw = origin[3:]

                            self.publish('pose3d', [self.xyz, origin[3:]])

                            print(self.time, "Origin received, internal position updated")
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
                            print (self.time, "app: True pose received: xyz=[%f,%f,%f], roll=%f, pitch=%f, yaw=%f" % (origin[0],origin[1],origin[2],self.nasa_roll, self.nasa_pitch, self.nasa_yaw))

                            print(self.time, "app: Final frame x=%d y=%d w=%d h=%d, nonblack=%d" % (data[1], data[2], data[3], data[4], data[5]))
                            angle_x = math.atan( (CAMERA_WIDTH / 2 - (img_x + img_w/2) ) / float(CAMERA_FOCAL_LENGTH))
                            angle_y = math.atan( (CAMERA_HEIGHT / 2 - (img_y + img_h/2) ) / float(CAMERA_FOCAL_LENGTH))

                            distance = self.interpolate_distance((img_w + img_h) / 2)
                            ax = self.nasa_yaw + angle_x
                            ay = self.nasa_pitch + angle_y + self.camera_angle
                            if self.use_gimbal:
                                # gimbal changes the actual angle dynamically so pitch needs to be offset
                                ay -= self.nasa_pitch

                            x, y, z = self.nasa_xyz
                            print("Using pose: xyz=[%f %f %f] orientation=[%f %f %f]" % (x, y, z, self.nasa_roll, self.nasa_pitch, self.nasa_yaw))
                            print("In combination with view angle %f %f and distance %f" % (ax, ay, distance))
                            ox = math.cos(ax) * math.cos(ay) * distance
                            oy = math.sin(ax) * math.cos(ay) * distance
                            oz = math.sin(ay) * distance
                            self.cubesat_location = (x+ox, y+oy, z+oz)
                            print (self.time, "app: Object offset calculated at: [%f %f %f]" % (ox, oy, oz))
                            print (self.time, "app: Reporting estimated object location at: [%f,%f,%f]" % (x+ox, y+oy, z+oz))

                            s = '%s %.2f %.2f %.2f\n' % (artifact_type, x+ox, y+oy, z+oz)
                            response = self.send_request('artf ' + s).decode("ascii") 

                            if response == 'ok':
                                print("app: Apriori object reported correctly")    
                                self.cubesat_success = True
                                # time to start looking for homebase; TODO queue 360 look around as base is somewhere near
                                self.object_reached("cubesat")
                                self.follow_object(['homebase'])
                            else:
                                print("app: Estimated object location incorrect, wait before continuing task; response: %s" % str(response))
                                self.last_attempt_timestamp = self.time
                        else:
                            print(self.time, "Origin request failed") # TODO: in future, we should try to find the cubesat again based on accurate position tracking
                            self.last_attempt_timestamp = self.time

                        # TODO: object was reached but not necessarily successfully reported;
                        # for now, just return driving to main which will either drive randomly with no additional purpose if homebase was previously found or will look for homebase
                        self.set_brakes(False)
                        self.current_driver = None
                        self.on_driving_control(timestamp, None) # do this last as it raises exception
                            
                        
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

                        
                elif math.isinf(self.homebase_final_approach_distance) and self.currently_following_object['object_type'] == 'homebase':
                    if bbox_size > 200:
                        if center_x >= 300 and center_x <= 340:
                            # object reached visually, keep moving forward
                            self.publish("desired_movement", [GO_STRAIGHT, 0, SPEED_ON])
                            print(self.time, "app: homebase final frame x=%d y=%d w=%d h=%d" % (data[1], data[2], data[3], data[4]))
                            self.homebase_final_approach_distance = 30.0
                            self.on_driving_control(timestamp, "homebase-final")
                            
                        elif center_x < 300: # close but wrong angle, turn in place left
                            self.publish("desired_movement", [0, 0, SPEED_ON])
                        elif center_x > 340: # close but wrong angle, turn in place right
                            self.publish("desired_movement", [0, 0, -SPEED_ON])
                    else:
                        if center_x < 300: # if homebase to the left, steer left
                            self.publish("desired_movement", [TURN_ON, 0, SPEED_ON])
                        elif center_x > 340:
                            self.publish("desired_movement", [-TURN_ON, 0, SPEED_ON])
                        else: # if within angle but object too small, keep going straight
                            self.publish("desired_movement", [GO_STRAIGHT, 0, SPEED_ON])

                elif self.currently_following_object['object_type'] == 'basemarker':
                    print(self.time, "app: basemarker identified")

                    if center_x < 300: # if marker to the left
                        self.basemarker_centered = False
                    elif center_x > 340:
                        self.basemarker_centered = False
                    else:
                        self.basemarker_centered = True
                        
                        
        
    def on_scan(self, timestamp, data):
        assert len(data) == 180
        super().on_scan(timestamp, data)


        if self.last_attempt_timestamp is not None and self.time - self.last_attempt_timestamp < ATTEMPT_DELAY:
            return
        
        # if was following an artefact but it disappeared, just go straight until another driver takes over
        if (
                self.currently_following_object['timestamp'] is not None and
                self.time - self.currently_following_object['timestamp'] > self.object_timeouts[self.currently_following_object['object_type']] and
                self.current_driver != "homebase-final" # if in final homebase step, we are not expected to be tracking homebase visually
                
        ):
            self.publish("desired_movement", [GO_STRAIGHT, 0, SPEED_ON])
            print (self.time, "No longer tracking %s" % self.currently_following_object['object_type'])
            self.currently_following_object['timestamp'] = None
            self.currently_following_object['object_type'] = None
            self.homebase_final_approach_distance = float("inf")
            self.basemarker_centered = False
            if self.current_driver != "basemarker": # do not change drivers when basemarker gets out of view because going around will bring it again
                self.on_driving_control(timestamp, None) # do this last as it raises exception

           
        # NASA sends 100 samples over 150 degrees
        # OSGAR sends 180 points, first and last 40 are zeros

# TODO: if too far from anything, revert to looking for homebase
        if not self.in_driving_recovery:

            midindex = len(data) // 2
            # 10 degrees left and right is 6-7 samples before and after the array midpoint
            straight_ahead_dist = min_dist(data[midindex-15:midindex+15])

            if 'homebase' in self.objects_to_follow and not math.isinf(self.homebase_final_approach_distance):
                print ("current distance: %f, min distance so far: %f" % (straight_ahead_dist , self.homebase_final_approach_distance))
                if straight_ahead_dist > self.homebase_final_approach_distance:
                    if self.homebase_increase_count < MAX_NR_OF_FARTHER_SCANS:
                        self.homebase_increase_count += 1
                    else:
                        self.homebase_increase_count = 0
                        # missed it, back to main driving loop in order to try again
                        print (self.time, "No longer tracking %s, distance increased" % self.currently_following_object['object_type'])
                        self.currently_following_object['timestamp'] = None
                        self.currently_following_object['object_type'] = None
                        self.homebase_final_approach_distance = float("inf")
                        self.basemarker_centered = False
                        self.on_driving_control(timestamp, None) # do this last as it possibly raises exception
                else:
                    self.homebase_increase_count = 0
                    self.homebase_final_approach_distance = straight_ahead_dist

                    if straight_ahead_dist < HOMEBASE_KEEP_DISTANCE:
                        self.publish("desired_movement", [0, 0, 0])

                        print ("app: final homebase distance %f: " % straight_ahead_dist)
                        self.homebase_final_approach_distance = float("inf")
                        self.object_reached('homebase')
                    else:
                        # keep going straight; for now this means do nothing, keep speed from previous step
                        self.currently_following_object['timestamp'] = self.time # freshen up timer as we are still following homebase

            # we expect that lidar bounces off of homebase as if it was a big cylinder, not taking into consideration legs, etc.

            if 'basemarker' in self.objects_to_follow:

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
                
                print (self.time, "app: Min dist front: %f, dist left=%f, right=%f" % (straight_ahead_dist, left_dist, right_dist))

                if len(self.basemarker_left_history) > 5 and left_dist > MAX_BASEMARKER_DISTANCE and right_dist > MAX_BASEMARKER_DISTANCE:
                    # lost contact with homebase, try approach again
                    print (self.time, "app: No longer going around homebase, distances inconsistent", self.basemarker_whole_scan_history)
                    self.currently_following_object['timestamp'] = None
                    self.currently_following_object['object_type'] = None
                    self.homebase_final_approach_distance = float("inf")
                    self.basemarker_centered = False
                    self.basemarker_right_history = self.basemarker_left_history = []
                    self.follow_object(['homebase'])
                    self.on_driving_control(timestamp, None) # do this last as it possibly raises exception
                    
                
    #            if left_dist < 9:
    #                print ("rover: right / left distance ratio: %f; centered: %r" % (right_dist / left_dist, self.basemarker_centered))
                if self.basemarker_centered and left_dist < 6 and abs(1.0 - right_dist / left_dist) < 0.04: # cos 20 = dist_r / dist _l is the max ratio in order to be at most 10 degrees off; also needs to be closer than 6m
                    self.publish("desired_movement", [0, -9000, 0])
                    self.set_brakes(True)
                    self.object_reached('basemarker')
                    return

                if left_dist > 10:
                    self.publish("desired_movement", [0, -9000, -SPEED_ON])
                elif right_dist > 10:
                    self.publish("desired_movement", [0, -9000, SPEED_ON])
                else:                    
                    if self.basemarker_radius is None:
                        self.basemarker_radius = HOMEBASE_KEEP_DISTANCE + HOMEBASE_RADIUS # ideal trajectory

                    if 1.0 - right_dist / left_dist > 0.2:
                        self.basemarker_radius -= 0.2
                    if 1.0 - right_dist / left_dist < -0.2:  #left is closer than right, need to increase the circle
                        self.basemarker_radius += 0.2

                    # negative radius turns to the right
                    self.publish("desired_movement", [-self.basemarker_radius, -9000, SPEED_ON])

        

    def run(self):
        try:
            print('Wait for definition of last_position and yaw')
            while self.last_position is None or self.yaw is None:
                self.update()  # define self.time
            print('done at', self.time)

            self.set_brakes(False)
            # some random manual starting moves to choose from
#            self.go_straight(-0.1, timeout=timedelta(seconds=20))
#            self.go_straight(-2, timeout=timedelta(seconds=20))
#            self.turn(math.radians(45), timeout=timedelta(seconds=20))
#            self.set_cam_angle(CAMERA_ANGLE_HOMEBASE)
#            self.follow_object(['basemarker'])
#            self.current_driver = 'basemarker'


            if SKIP_CUBESAT_SUCCESS:
                # skip cubesat
                self.cubesat_success = True
                self.follow_object(['homebase'])
            else:
                # regular launch
                self.follow_object(['cubesat', 'homebase'])

#            self.publish("desired_movement", [10, 9000, 10])
#            self.wait(timedelta(seconds=10))
                                    
            last_walk_start = 0.0
            start_time = self.time
            while self.time - start_time < timedelta(minutes=40):
                additional_turn = 0
                last_walk_start = self.time

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
                    print(self.time, "Turn interrupted by driver: %s" % e)
                    continue
                except (VirtualBumperException, LidarCollisionException):
                    self.inException = True
                    self.set_cam_angle(CAMERA_ANGLE_DRIVING)
                    print(self.time, "Turn Virtual Bumper!")
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
                    print(self.time, repr(e))
                    last_walk_end = self.time
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
                    print(self.time, "Turn Virtual Bumper!")
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
