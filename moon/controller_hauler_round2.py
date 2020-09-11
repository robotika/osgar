"""
  Space Robotics Challenge 2
"""

from scipy import spatial
import numpy as np
import math
from datetime import timedelta
from statistics import median
import traceback
import sys

from osgar.bus import BusShutdownException

from moon.controller import ps, SpaceRoboticsChallenge, MoonException, ChangeDriverException, VirtualBumperException, min_dist, LidarCollisionException, LidarCollisionMonitor
from osgar.lib.quaternion import euler_zyx
from osgar.lib.virtual_bumper import VirtualBumper
from osgar.lib.mathex import normalizeAnglePIPI
from moon.moonnode import CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_FOCAL_LENGTH, CAMERA_BASELINE

TURN_ON = 12 # radius of circle when turning
GO_STRAIGHT = float("inf")
EXCAVATOR_DRIVING_GAP = 1.8 # can't be further or every bump will look like the excavator on lidar given the camera must be tilted down so that rover is visible up close
EXCAVATOR_ONHOLD_GAP = 3 # can't be further or every bump will look like the excavator on lidar given the camera must be tilted down so that rover is visible up close
EXCAVATOR_DIGGING_GAP = 0.55 # we should see the arm in the middle, not the back of the rover
DISTANCE_TOLERANCE = 0.3

class ExcavatorLostException(MoonException):
    pass

class SpaceRoboticsChallengeHaulerRound2(SpaceRoboticsChallenge):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register("desired_movement")
        self.robot_name = "hauler_1"

        self.rover_angle = 0.0
        self.use_gimbal = True
        self.vslam_reset_at = None
        self.goto = None
        self.finish_visually = False
        self.arrived_message_sent = False
        self.objects_in_view = {}
        self.bin_content = None
        self.excavator_yaw = None
        self.default_effort_level = 1100
        self.rover_distance = 15
        self.min_scan_distance = 15
        self.scan_driving = False
        self.cam_lowered = False
        self.driving_mode = None
        self.aligned_at = None
        self.target_excavator_distance = EXCAVATOR_DRIVING_GAP
        self.scan_driving_phase = "yaw"
        self.scan_driving_phase_start = None
        self.excavator_waiting = False
        self.turnto_angle = None
        self.arrival_send_requested_at = None
        self.set_yaw = None
        self.full_360_objects = {}
        self.excavator_waiting_start = None

    def vslam_reset_time(self, response):
        self.vslam_reset_at = self.sim_time

    def on_osgar_broadcast(self, data):

        message_target = data.split(" ")[0]
        if message_target == self.robot_name:
            print(self.sim_time, self.robot_name, "Received external_command: %s" % str(data))
            command = data.split(" ")[1]
            self.driving_mode = command
            self.set_brakes(False)

            if command == "goto":
                # x, y, heading
                self.goto = [float(n) for n in data.split(" ")[2:5]]
                self.send_request('vslam_reset', self.vslam_reset_time)
            elif command == "approach":
                self.arrived_message_sent = False
                self.excavator_yaw = float(data.split(" ")[2])
            elif command == "align":
                self.target_excavator_distance = EXCAVATOR_DRIVING_GAP
                self.arrived_message_sent = False
                self.excavator_yaw = float(data.split(" ")[2])
            elif command == "follow":
                self.target_excavator_distance = EXCAVATOR_DRIVING_GAP
                self.arrived_message_sent = False
            elif command == "onhold":
                self.target_excavator_distance = EXCAVATOR_ONHOLD_GAP
                self.arrived_message_sent = False
            elif command == "request_true_pose":
                def forward_true_pose(result):
                    if result.split()[0] == 'origin':
                        origin = [float(x) for x in result.split()[1:]]
                        initial_quat = origin[3:]
                        initial_rpy = euler_zyx(initial_quat) # note: this is not in roll, pitch, yaw order
                        self.yaw_offset = self.yaw + self.yaw_offset - initial_rpy[0]
                    self.send_request('external_command excavator_1 hauler_true_pose %s' % result)
                self.send_request('request_origin', forward_true_pose)
            elif command == "turnto":
                self.turnto_angle = float(data.split(" ")[2])
            elif command == "set_yaw":
                self.set_yaw = float(data.split(" ")[2])
            else:
                print(self.sim_time, self.robot_name, "Invalid broadcast command")

    def get_extra_status(self):
        return "Bin: " + str([[a,b] for a,b in zip(self.bin_content[0], self.bin_content[1])])

    def on_bin_info(self, data):
        self.bin_content = data

    def look_for_rover(self, direction=1):
        print(self.sim_time, self.robot_name, "Starting to look for excavator")
        while True:
            try:
                with LidarCollisionMonitor(self):
                    self.turn(math.copysign(math.radians(self.rand.randrange(90,270)), direction), timeout=timedelta(seconds=20))
                    self.turn(math.radians(360), timeout=timedelta(seconds=40))
                    self.go_straight(20.0, timeout=timedelta(minutes=2))
            except ChangeDriverException as e:
                print(self.sim_time, self.robot_name, "Excavator found, following")
                self.publish("desired_movement", [0, 0, 0])
                raise
            except (VirtualBumperException, LidarCollisionException)  as e:
                self.inException = True
                print(self.sim_time, self.robot_name, "Lidar or Virtual Bumper!")
                try:
                    self.go_straight(-3, timeout=timedelta(seconds=20))
                    deg_angle = self.rand.randrange(-180, -90)
                    self.turn(math.radians(deg_angle), timeout=timedelta(seconds=10))
                except:
                    self.publish("desired_movement", [0, 0, 0])
                self.inException = False
            except MoonException as e:
                print(self.sim_time, self.robot_name, "MoonException while looking for rover, restarting turns")
                traceback.print_exc(file=sys.stdout)



    def run(self):

        try:
            self.wait_for_init()
            #self.wait(timedelta(seconds=5))
            self.set_light_intensity("0.2")
            self.set_cam_angle(-0.1)
            self.use_gimbal = True

            self.set_brakes(True)
            if False:
                # point wheels downwards and wait until on flat terrain or timeout
                start_drifting = self.sim_time
                while self.sim_time - start_drifting < timedelta(seconds=20) and (abs(self.pitch) > 0.05 or abs(self.roll) > 0.05):
                    try:
                        attitude = math.asin(math.sin(self.roll) / math.sqrt(math.sin(self.pitch)**2+math.sin(self.roll)**2))
                        if self.debug:
                            print(self.sim_time, self.robot_name, "Vehicle attitude: %.2f" % attitude)
                        self.publish("desired_movement", [GO_STRAIGHT, math.degrees(attitude * 100), 0.1])
                        self.wait(timedelta(milliseconds=100))
                    except MoonException as e:
                        print(self.sim_time, self.robot_name, "Exception while waiting for excavator to settle: ", str(e))
                self.publish("desired_movement", [0, 0, 0])

            while True:
                while self.driving_mode is None: # waiting for align command
                    try:
                        self.wait(timedelta(seconds=1))
                    except MoonException as e:
                        print(self.sim_time, self.robot_name, "Exception while waiting to be invited to look for excavator", str(e))
                break

            self.set_brakes(False)
            self.finish_visually = True

            # FIND EXCAVATOR
            while True:
                try:
                    self.excavator_waiting = True
                    with LidarCollisionMonitor(self):
                        while True:
                            self.turn(math.radians(self.rand.randrange(90,270)), timeout=timedelta(seconds=20))
                            self.turn(math.radians(360), timeout=timedelta(seconds=40))
                            self.go_straight(20.0, timeout=timedelta(minutes=2))
                except ChangeDriverException as e:
                    print(self.sim_time, self.robot_name, "Excavator found, following")
                    self.publish("desired_movement", [0, 0, 0])
                except (VirtualBumperException, LidarCollisionException)  as e:
                    self.inException = True
                    print(self.sim_time, self.robot_name, "Lidar or Virtual Bumper!")
                    try:
                        self.go_straight(-3, timeout=timedelta(seconds=20))
                        deg_angle = self.rand.randrange(-180, -90)
                        self.turn(math.radians(deg_angle), timeout=timedelta(seconds=10))
                    except:
                        self.publish("desired_movement", [0, 0, 0])
                    self.inException = False
                    continue
                except MoonException as e:
                    print(self.sim_time, self.robot_name, "MoonException while looking for rover, restarting turns")
                    traceback.print_exc(file=sys.stdout)
                    continue

                # follow to ONHOLD distance
                self.driving_mode = "onhold"
                self.target_excavator_distance = EXCAVATOR_ONHOLD_GAP

                # driving towards excavator, wait until hauler is in position
                while abs(self.target_excavator_distance - self.rover_distance) > DISTANCE_TOLERANCE:
                    try:
                        self.wait(timedelta(milliseconds=200))
                    except ExcavatorLostException:
                        print(self.sim_time, self.robot_name, "Excavator lost while waiting to reach desired location, starting to look again")
                        break # look again
                    except MoonException:
                        print(self.sim_time, self.robot_name, "Exception while waiting to reach desired location, waiting on")

                if abs(self.target_excavator_distance - self.rover_distance) <= DISTANCE_TOLERANCE:
                    break # distance reached

            self.publish("desired_movement", [0, 0, 0])
            self.set_brakes(True)
            print(self.sim_time, self.robot_name, "Sending arrived message to excavator")
            self.arrived_message_sent = True
            self.send_request('external_command excavator_1 arrived %.2f' % 0.0)

            # turn lights off while waiting for excavator to rotate away from hauler
            # prevents potentially illuminating nearby processing plant too much and causing mis-detections
            self.set_light_intensity("0.0")
            while self.set_yaw is None:
                try:
                    self.wait(timedelta(seconds=1))
                except MoonException:
                    print(self.sim_time, self.robot_name, "Exception while waiting for yaw alignment readiness, waiting on")

            self.set_brakes(False)
            self.set_light_intensity("0.2")

            print(self.sim_time, self.robot_name, "Sending arrived message to excavator")
            self.send_request('external_command excavator_1 arrived %.2f' % self.set_yaw)
            #self.set_yaw = None


            # self.finish_visually = True

            # self.excavator_waiting = True

            self.virtual_bumper = VirtualBumper(timedelta(seconds=30), 0.1) # TODO: need generous timeout because turning around or sideways moves are not sensed by bumper

            # 3 modes: 1) going to location 2) following rover visually 3) waiting for instructions to go to location

            while True:


                if self.goto is not None:
                    try:
                        with LidarCollisionMonitor(self, 1000):
                            angle_diff = self.get_angle_diff(self.goto, 1)
                            self.turn(angle_diff, timeout=timedelta(seconds=40))
                            self.go_to_location(self.goto, self.default_effort_level, full_turn=False, avoid_obstacles_close_to_destination=True, timeout=timedelta(minutes=2))
                            self.turn(normalizeAnglePIPI(self.goto[2] - self.yaw), timeout=timedelta(seconds=40))
                        self.send_request('external_command excavator_1 arrived %.2f' % self.yaw)
                        self.goto = None
                        self.finish_visually = True
                        self.set_cam_angle(-0.1)
                        self.set_light_intensity("0.2")

                    except ChangeDriverException as e:
                        print(self.sim_time, self.robot_name, "Driver changed during goto?")
                    except LidarCollisionException as e: #TODO: long follow of obstacle causes loss, go along under steeper angle
                        print(self.sim_time, self.robot_name, "Lidar")
                        self.inException = True
                        self.lidar_drive_around()
                        self.inException = False
                        continue
                    except VirtualBumperException as e:
                        self.send_speed_cmd(0.0, 0.0)
                        print(self.sim_time, self.robot_name, "Bumper")
                        self.inException = True
                        self.go_straight(-1) # go 1m in opposite direction
                        self.drive_around_rock(math.copysign(4, 1 if self.rand.getrandbits(1) == 0 else -1)) # assume 6m the most needed
                        self.inException = False
                        continue

                try:
                    if self.excavator_waiting:
                        self.turn(math.copysign(math.radians(self.rand.randrange(90,270)), self.rover_angle), timeout=timedelta(seconds=20))
                        self.turn(math.radians(360), timeout=timedelta(seconds=40))
                        self.go_straight(20.0, timeout=timedelta(seconds=40))
                    else:
                        if self.rover_distance > 5 and self.driving_mode != "approach":
                            with LidarCollisionMonitor(self, 1000):
                                self.wait(timedelta(seconds=1))
                        else:
                                self.wait(timedelta(seconds=1))
                except LidarCollisionException as e: #TODO: long follow of obstacle causes loss, go along under steeper angle
                    print(self.sim_time, self.robot_name, "Lidar while following/waiting, distance: %.1f" % self.rover_distance)
                    self.excavator_waiting = True
                    self.send_request('external_command excavator_1 wait')
                    self.inException = True

                    try:
                        self.lidar_drive_around()  # TODO: if we lose visual of excavator, will keep going and never find, maybe turn 360
                    except ExcavatorLostException as e:
                        self.excavator_waiting = True
                        self.send_request('external_command excavator_1 wait')
                    except MoonException as e:
                        print(self.sim_time, self.robot_name, "Exception while handling Lidar", str(e))
                        traceback.print_exc(file=sys.stdout)

                    self.inException = False
                    self.send_speed_cmd(0.0, 0.0)
                except ChangeDriverException as e:
                    print(self.sim_time, self.robot_name, "Excavator found, continuing visual driving")
                except VirtualBumperException as e: # if bumper while following (e.g, blocked by a rock)
                    if self.arrived_message_sent: # if while waiting for loading, ignore
                        continue
                    self.send_speed_cmd(0.0, 0.0)
                    print(self.sim_time, self.robot_name, "Bumper")
                    self.inException = True

                    try:
                        self.go_straight(-1) # go 1m in opposite direction
                        self.drive_around_rock(math.copysign(4, 1 if self.rand.getrandbits(1) == 0 else -1)) # assume 6m the most needed
                    except ExcavatorLostException as e:
                        self.excavator_waiting = True
                        self.send_request('external_command excavator_1 wait')

                    self.inException = False
                except ExcavatorLostException as e:
                    self.excavator_waiting = True
                    self.send_request('external_command excavator_1 wait')

        except BusShutdownException:
            pass

        print("HAULER END")

    def on_vslam_pose(self, data):
        super().on_vslam_pose(data)

        if self.sim_time is None or self.last_position is None or self.yaw is None:
            return

        if self.vslam_reset_at is not None and self.sim_time - self.vslam_reset_at > timedelta(seconds=3) and not math.isnan(data[0][0]) and self.tf['vslam']['trans_matrix'] is None:
            # request origin and start tracking in correct coordinates as soon as first mapping lock occurs
            # TODO: another pose may arrive while this request is still being processed (not a big deal, just a ROS error message)
            self.send_request('request_origin', self.register_origin)


    def on_artf(self, data):

        artifact_type = data[0]
        center_x = data[1] + data[3] / 2
        center_y = data[2] + data[4] / 2
        img_x1 = data[1]
        img_x2 = data[1] + data[3]
        nr_pixels = data[5]
        artf_distance = data[6] # in meters

        if self.sim_time is None:
            return


        #if self.set_yaw is not None:
        #    if artifact_type not in self.full_360_objects.keys():
        #        self.full_360_objects[artifact_type] = []
        #    self.full_360_objects[artifact_type].append(self.yaw)

        if not self.finish_visually:
            return

        self.objects_in_view[artifact_type] = {
            "expiration": self.sim_time + timedelta(milliseconds=1000),
            "distance": artf_distance
        }

        if artifact_type == "rover" or artifact_type == "excavator_arm":
            if self.excavator_waiting_start is None:
                self.excavator_waiting_start = self.sim_time

            # do not act on the first matching frame, keep moving in the same manner a bit longer to get the desired view more established
            if self.excavator_waiting and self.sim_time - self.excavator_waiting_start > timedelta(milliseconds=800):
                self.send_request('external_command excavator_1 resume')
                self.excavator_waiting = False
                self.excavator_waiting_start = None
                raise ChangeDriverException(data)

            # TODO: maybe just use min of stereo cam distances?
            if artf_distance < 1.3:
                self.rover_distance = self.min_scan_distance # for stereo camerra distance less than 1.3m, rely on lidar for distance
            elif len(self.median_scan) > 0:
                self.rover_distance = max(min(self.median_scan)/1000.0, min([self.objects_in_view[a]['distance'] for a in self.objects_in_view])) # TODO: ?????

        if artifact_type == "excavator_arm" or (artifact_type == "rover" and "excavator_arm" not in self.objects_in_view.keys()):
            screen_x1 = (CAMERA_WIDTH / 2 - img_x1)
            screen_x2 = (CAMERA_WIDTH / 2 - img_x2)
            angle_x1 = math.atan( screen_x1 / float(CAMERA_FOCAL_LENGTH))
            angle_x2 = math.atan( screen_x2 / float(CAMERA_FOCAL_LENGTH))
            angle_offset = math.atan( (CAMERA_BASELINE / 2) / (self.rover_distance + 1.1))
            self.rover_angle = (angle_x1 + angle_x2) / 2 + angle_offset


#            if (self.debug):
#                print (self.sim_time, self.robot_name, "Rover angle: %.2f dist: %d" % (angle_x, self.rover_distance))


        if not self.inException and not self.arrived_message_sent and not self.scan_driving:
            if self.rover_distance > 6 and self.scan_distance_to_obstacle < 4000:
                turn = self.get_avoidance_turn()
                self.publish("desired_movement", [turn, 0, self.default_effort_level])
                if (self.debug):
                    print(self.sim_time, self.robot_name, "Visual drive: avoiding obstacle, obstacle dist: %d, rover dist: %.1f" % (self.scan_distance_to_obstacle, self.rover_distance))
            elif (
                artifact_type == "excavator_arm" or (artifact_type == "rover" and "excavator_arm" not in self.objects_in_view.keys())
            ):
                # drive visually until both in 'arrival' mode and at target distance
                self.last_rover_timestamp = self.sim_time

                direction = 1 if self.target_excavator_distance < self.rover_distance else -1

                # if hauler is within 1m of something (presumably excavator), regardless of target distance from excavator, move sideways to center
                # if bbox center in left or right quarter, turn in place
                # else if bbox off center turn while moving
                # else (bbox in center) go straight

                # TODO: if hauler approaches on a diagonal, it may not get close enough to switch to lidar driving
                speed = self.default_effort_level
                if self.set_yaw is None: # skip adjustments before the yaw is set, need to hit excavator with a little force to align yaws accurately
                    if self.rover_distance < 3:
                        speed = 0.5 * self.default_effort_level
                    elif self.rover_distance < self.target_excavator_distance + 2:
                        speed = 0.5 * self.default_effort_level
                    elif self.rover_distance > self.target_excavator_distance + 3:
                        speed = 1.2 * self.default_effort_level

                # if too close, just go straight back first
                if self.driving_mode != "approach" and abs(self.target_excavator_distance - self.rover_distance) < DISTANCE_TOLERANCE:
                    # on approach, tolerance is not good enough, needs to be close than; pause and conclusion of approach will be performed by scan_driving
                    self.publish("desired_movement", [0, 0, 0])
                    if (self.debug):
                        print(self.sim_time, self.robot_name, "Visual drive: within range, pausing, rover distance: %d,%.1f" % (self.scan_distance_to_obstacle, self.rover_distance))

                elif self.target_excavator_distance > self.rover_distance:
                    self.publish("desired_movement", [GO_STRAIGHT, 0, -speed])
                    if (self.debug):
                        print(self.sim_time, self.robot_name, "Visual drive: too close, going straight back, rover distance: %d,%.1f" % (self.scan_distance_to_obstacle, self.rover_distance))

                elif center_x < CAMERA_WIDTH/4:
                    if (self.debug):
                        print(self.sim_time, self.robot_name, "Visual drive: Turning sharply left, obstacle dist: %d, rover dist: %.1f" % (self.scan_distance_to_obstacle, self.rover_distance))
                    self.publish("desired_movement", [direction * TURN_ON/3, 0, direction * speed])
                elif center_x < (CAMERA_WIDTH/2 - 20):
                    if (self.debug):
                        print(self.sim_time, self.robot_name, "Visual drive: steering left, rover distance: %d,%.1f" % (self.scan_distance_to_obstacle, self.rover_distance))
                    self.publish("desired_movement", [direction * TURN_ON, 0, direction * speed])
                elif center_x > 3*CAMERA_WIDTH/4:
                    if (self.debug):
                        print(self.sim_time, self.robot_name, "Visual drive: Turning sharply right, rover distance: %d,%.1f" % (self.scan_distance_to_obstacle, self.rover_distance))
                    self.publish("desired_movement", [direction * -TURN_ON/3, 0, direction * speed])
                elif center_x > (CAMERA_WIDTH/2 + 20):
                    if (self.debug):
                        print(self.sim_time, self.robot_name, "Visual drive: steering right, rover distance: %d,%.1f" % (self.scan_distance_to_obstacle, self.rover_distance))
                    self.publish("desired_movement", [direction * -TURN_ON, 0, direction * speed])
                else:
                    if (self.debug):
                        print(self.sim_time, self.robot_name, "Visual drive: going straight, rover distance: %d,%.1f" % (self.scan_distance_to_obstacle, self.rover_distance))
                    self.publish("desired_movement", [GO_STRAIGHT, 0, direction * speed])


    def on_scan(self, data):
        assert len(data) == 180
        super().on_scan(data)

        if len(self.median_scan) > 0:
            self.min_scan_distance = min(self.median_scan) / 1000.0

        delete_in_view = [artf for artf in self.objects_in_view if self.objects_in_view[artf]['expiration'] < self.sim_time]
        for artf in delete_in_view:
            del self.objects_in_view[artf]

        if "rover" not in self.objects_in_view.keys() and "excavator_arm" not in self.objects_in_view.keys() and self.rover_distance < 15:
            self.rover_distance = 15
            if (self.debug):
                print(self.sim_time, self.robot_name, "Expiring rover view and resetting distance to 15m")
            if not self.arrived_message_sent:
                raise ExcavatorLostException()

        if not self.finish_visually:
            return

        # once distance reached, go on another 200ms to get real tight
        if self.arrival_send_requested_at is not None and self.sim_time - self.arrival_send_requested_at > timedelta(milliseconds=200):
            self.publish("desired_movement", [0, 0, 0])

        if self.arrival_send_requested_at is not None and self.sim_time - self.arrival_send_requested_at > timedelta(milliseconds=1500):
            self.arrival_send_requested_at = None
            if self.set_yaw is not None:
                print(self.sim_time, self.robot_name, "Internal yaw: %.2f, set yaw: %.2f, yaw offset: %.2f" % (self.yaw, self.set_yaw, self.yaw - self.set_yaw))
                self.yaw_offset = self.yaw - self.set_yaw
                self.set_yaw = None
            print(self.sim_time, self.robot_name, "Sending arrived message to excavator")
            self.send_request('external_command excavator_1 arrived %.2f' % self.yaw)


        # TODO: avoid obstacles when following (too tight turns)

        if ('rover' in self.objects_in_view.keys() or 'excavator_arm' in self.objects_in_view.keys()) and (self.driving_mode == "approach" or self.driving_mode == "align" or self.driving_mode == "turnto") and not self.arrived_message_sent and not self.inException:
            # TODO: sometimes we are near rover but don't see its distance
            # other times, we see a rover and its distance but stuck behind a rock
            if self.excavator_yaw is None: # within requested distance and no outstanding yaw, report
                if min(self.rover_distance, self.min_scan_distance) < self.target_excavator_distance:
                    self.set_brakes(True)
                    self.arrival_send_requested_at = self.sim_time
                    self.arrived_message_sent = True
                    self.excavator_yaw = None
                    self.scan_driving = False
                    if self.debug:
                        print(self.sim_time, self.robot_name, "Arrival handling: distance %.1f, yaw: %.2f, rover angle: %.2f; stopping" % (self.rover_distance, self.yaw, self.rover_angle))

            if self.scan_driving or (abs(self.rover_distance - self.target_excavator_distance) < DISTANCE_TOLERANCE and self.excavator_yaw is not None): # within 1m from target, start adjusting angles
                if not self.scan_driving:
                    if self.set_yaw is None: # do not stop when doing initial excavator approach to bump and align better
                        self.publish("desired_movement", [0, 0, 0])
                    self.scan_driving = True

                diff = normalizeAnglePIPI(self.yaw - self.excavator_yaw)
                if (self.debug):
                    print(self.sim_time, self.robot_name, "Arrival handling: yaw difference: (%.2f - %.2f)= %.2f, rover angle: %.2f" % (self.yaw, self.excavator_yaw, diff, self.rover_angle))


                if self.scan_driving_phase == "yaw":
                    if self.scan_driving_phase_start is None:
                        self.scan_driving_phase_start = self.sim_time
                    if abs(diff) > 0.12 and self.set_yaw is None and self.sim_time - self.scan_driving_phase_start < timedelta(seconds=30): # 10deg, don't bother otherwise; also, don't attempt to align before true pose is known
                            if (self.debug):
                                print(self.sim_time, self.robot_name, "Arrival handling: moving on a curve")
                            self.publish("desired_movement", [-8, -9000, math.copysign(0.5 * self.default_effort_level, diff)])
                            #TODO: keep doing until diff=0; however we are bound to overshoot so maybe leave as is
                    else:
                         self.scan_driving_phase = "behind"

                if self.scan_driving_phase == "behind":
                    if abs(self.rover_angle) > 0.12 and self.set_yaw is None and self.sim_time - self.scan_driving_phase_start < timedelta(seconds=50):
                        if (self.debug):
                            print(self.sim_time, self.robot_name, "Arrival handling: moving sideways")
                        self.publish("desired_movement", [GO_STRAIGHT, -9000, -math.copysign(0.5 * self.default_effort_level, self.rover_angle)])
                        #TODO: keep doing until angle=0
                    else:
                        self.scan_driving_phase = "waiting"

                if self.scan_driving_phase == "waiting":
                    if (self.debug):
                        print(self.sim_time, self.robot_name, "Arrival handling: final wait to straighten wheels")
                    if self.aligned_at is None:
                        self.aligned_at = self.sim_time

                    if self.sim_time - self.aligned_at > timedelta(milliseconds=2500) or self.set_yaw is not None:
                        if self.driving_mode == "approach":
                            self.target_excavator_distance = EXCAVATOR_DIGGING_GAP
                        self.excavator_yaw = None
                        self.scan_driving = False
                        self.aligned_at = None
                        self.scan_driving_phase = "yaw"
                        self.scan_driving_phase_start = None
                    else:
                        self.publish("desired_movement", [0, 0, 0]) # going straight doesn't wait for steering so we have to wait here

#            else:
#                if (self.debug):
#                    print(self.sim_time, self.robot_name, "Arrival handling: going forward")
#                self.publish("desired_movement", [GO_STRAIGHT, 0, self.default_effort_level])



#        self.rover_angle = rover_center_angle(data)

# vim: expandtab sw=4 ts=4
