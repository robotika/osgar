"""
  Space Robotics Challenge 2
"""

from scipy import spatial
import numpy as np
import math
from datetime import timedelta
from statistics import median

from osgar.bus import BusShutdownException

from moon.controller import ps, SpaceRoboticsChallenge, ChangeDriverException, VirtualBumperException, min_dist, LidarCollisionException, LidarCollisionMonitor
from osgar.lib.quaternion import euler_zyx
from osgar.lib.virtual_bumper import VirtualBumper
from osgar.lib.mathex import normalizeAnglePIPI

CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FOCAL_LENGTH = 381

TURN_ON = 12 # radius of circle when turning
GO_STRAIGHT = float("inf")
EXCAVATOR_DRIVING_GAP = 2.200 # can't be further or every bump will look like the excavator on lidar given the camera must be tilted down so that rover is visible up close
EXCAVATOR_DIGGING_GAP = 0.8 # we should see the arm in the middle, not the back of the rover

class SpaceRoboticsChallengeHaulerRound2(SpaceRoboticsChallenge):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register("desired_movement")
        self.robot_name = "hauler_1"

        self.rover_angle = None
        self.use_gimbal = False
        self.vslam_reset_at = None
        self.goto = None
        self.finish_visually = False
        self.arrived_message_sent = False
        self.objects_in_view = {}
        self.bin_content = None
        self.excavator_yaw = None
        self.default_effort_level = 1100
        self.rover_distance = 15
        self.scan_driving = False
        self.cam_lowered = False
        self.driving_mode = "follow"
        self.target_excavator_distance = EXCAVATOR_DRIVING_GAP


    def on_osgar_broadcast(self, data):
        def vslam_reset_time(response):
            self.vslam_reset_at = self.sim_time

        message_target = data.split(" ")[0]
        if message_target == self.robot_name:
            print(self.sim_time, self.robot_name, "Received external_command: %s" % str(data))
            command = data.split(" ")[1]
            self.driving_mode = command
            if command == "goto":
                # x, y, heading
                self.goto = [float(n) for n in data.split(" ")[2:5]]
                self.send_request('vslam_reset', vslam_reset_time)
            elif command == "approach":
                self.arrived_message_sent = False
                self.excavator_yaw = float(data.split(" ")[2])
            elif command == "backout":
                self.arrived_message_sent = False
                self.target_excavator_distance = EXCAVATOR_DRIVING_GAP
            elif command == "follow":
                self.target_excavator_distance = EXCAVATOR_DRIVING_GAP
                self.arrived_message_sent = False
            else:
                print(self.sim_time, self.robot_name, "Invalid broadcast command")

    def get_extra_status(self):
        return "Bin: " + str([[a,b] for a,b in zip(self.bin_content[0], self.bin_content[1])])

    def on_bin_info(self, data):
        self.bin_content = data

    def run(self):

        try:
            self.wait_for_init()
            #self.wait(timedelta(seconds=5))
            self.set_light_intensity("0.4")
            self.set_cam_angle(0.0) # when following visually, look straight

            self.set_brakes(False)

            while not self.true_pose:
                self.wait(timedelta(seconds=1))

            self.virtual_bumper = VirtualBumper(timedelta(seconds=10), 0.1) # need generous timeout because sideways moves are not sensed by bumper

            # 3 modes: 1) going to location 2) following rover visually 3) waiting for instructions to go to location

            while True:


                if self.goto is not None:
                    try:
                        with LidarCollisionMonitor(self, 1000):
                            angle_diff = self.get_angle_diff(self.goto, 1)
                            self.turn(angle_diff)
                            self.go_to_location(self.goto, self.default_effort_level, full_turn=False, timeout=timedelta(minutes=2))
                            self.turn(normalizeAnglePIPI(self.goto[2] - self.yaw))
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
                        self.drive_around_rock(6) # assume 6m the most needed
                        self.inException = False
                        continue
                try:
                    if self.rover_distance > 5 and self.driving_mode != "approach":
                        with LidarCollisionMonitor(self, 1000):
                            self.wait(timedelta(seconds=1))
                    else:
                            self.wait(timedelta(seconds=1))
                except LidarCollisionException as e: #TODO: long follow of obstacle causes loss, go along under steeper angle
                    print(self.sim_time, self.robot_name, "Lidar while following/waiting, distance: %.1f" % self.rover_distance)
                    self.inException = True
                    self.lidar_drive_around()
                    self.inException = False
                    continue
                except ChangeDriverException as e:
                    print(self.sim_time, self.robot_name, "Driver changed to go to location")
                except VirtualBumperException as e: # if bumper while following (e.g, blocked by a rock)
                    if self.arrived_message_sent: # if while waiting for loading, ignore
                        continue
                    self.send_speed_cmd(0.0, 0.0)
                    print(self.sim_time, self.robot_name, "Bumper")
                    self.inException = True
                    self.go_straight(-1) # go 1m in opposite direction
                    self.drive_around_rock(4) # assume 6m the most needed
                    self.inException = False
                    continue



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

        if not self.finish_visually:
            return

        artifact_type = data[0]
        center_x = data[1] + data[3] / 2
        center_y = data[2] + data[4] / 2
        img_x1 = data[1]
        img_x2 = data[1] + data[3]
        nr_pixels = data[5]
        artf_distance = data[6] # in meters

        if self.sim_time is None:
            return

        self.objects_in_view[artifact_type] = {
            "expiration": self.sim_time + timedelta(milliseconds=1000)
        }

        if artifact_type == "rover":
            if artf_distance < 1.3:
                self.rover_distance = self.scan_distance_to_obstacle/1000.0 # for distance less than 1.3m, rely on lidar for distance
            else:
                self.rover_distance = max(min(self.median_scan)/1000.0, artf_distance)

            screen_x1 = (CAMERA_WIDTH / 2 - img_x1)
            screen_x2 = (CAMERA_WIDTH / 2 - img_x2)
            angle_x1 = math.atan( screen_x1 / float(CAMERA_FOCAL_LENGTH))
            angle_x2 = math.atan( screen_x2 / float(CAMERA_FOCAL_LENGTH))
            self.rover_angle = (angle_x1 + angle_x2) / 2 # does not work at close proximity as only a portion of the robot may be visible centering around that portion

            #if self.rover_distance < 2500 and not self.cam_lowered: #if closer to rover than 2m, look down to help lidar
            #    self.set_cam_angle(-0.1)
            #    self.cam_lowered = True
            #elif self.rover_distance >= 2700 and self.cam_lowered:
            #    self.set_cam_angle(-0.1)
            #    self.cam_lowered = False


#            if (self.debug):
#                print (self.sim_time, self.robot_name, "Rover angle: %.2f dist: %d" % (angle_x, self.rover_distance))


        if artifact_type == "rover" and not self.inException and not self.arrived_message_sent and not self.scan_driving:
            # drive visually until both in 'arrival' mode and at target distance
            self.last_rover_timestamp = self.sim_time

            direction = 1 if self.target_excavator_distance < self.rover_distance else -1

            # if hauler is within 1m of something (presumably excavator), regardless of target distance from excavator, move sideways to center
            if False and self.rover_distance < 1 and abs(self.rover_angle) > 0.2:
                self.publish("desired_movement", [GO_STRAIGHT, -math.copysign(7500, self.rover_angle), -self.default_effort_level])
#                if self.rover_angle > 0.2:
#                    self.publish("desired_movement", [0, 0, self.default_effort_level])
#                elif self.rover_angle < -0.2:
#                    self.publish("desired_movement", [0, 0, -self.default_effort_level])
#                else:
#                    # centered and between 0.5 and 2m distant
#                    self.publish("desired_movement", [0, 0, 0])

            else:
                # if bbox center in left or right quarter, turn in place
                # else if bbox off center turn while moving
                # else (bbox in center) go straight

                # TODO: if hauler approaches on a diagonal, it may not get close enough to switch to lidar driving
                if center_x < CAMERA_WIDTH/4:
                    if (self.debug):
                        print(self.sim_time, self.robot_name, "Visual drive: Turning in place left, obstacle dist: %d, rover dist: %.1f" % (self.scan_distance_to_obstacle, self.rover_distance))
                    self.publish("desired_movement", [TURN_ON/2, 0, self.default_effort_level])
                elif center_x < (CAMERA_WIDTH/2 - 20):
                    if (self.debug):
                        print(self.sim_time, self.robot_name, "Visual drive: steering left, rover distance: %d,%.1f" % (self.scan_distance_to_obstacle, self.rover_distance))
                    self.publish("desired_movement", [TURN_ON, 0, direction * self.default_effort_level])
                elif center_x > 3*CAMERA_WIDTH/4:
                    if (self.debug):
                        print(self.sim_time, self.robot_name, "Visual drive: Turning in place right, rover distance: %d,%.1f" % (self.scan_distance_to_obstacle, self.rover_distance))
                    self.publish("desired_movement", [-TURN_ON/2, 0, self.default_effort_level])
                elif center_x > (CAMERA_WIDTH/2 + 20):
                    if (self.debug):
                        print(self.sim_time, self.robot_name, "Visual drive: steering right, rover distance: %d,%.1f" % (self.scan_distance_to_obstacle, self.rover_distance))
                    self.publish("desired_movement", [-TURN_ON, 0, direction * self.default_effort_level])
                else:
                    speed = self.default_effort_level if self.rover_distance > 5 else 0.5 * self.default_effort_level
                    if (self.debug):
                        print(self.sim_time, self.robot_name, "Visual drive: going straight, rover distance: %d,%.1f" % (self.scan_distance_to_obstacle, self.rover_distance))
                    self.publish("desired_movement", [GO_STRAIGHT, 0, direction * speed])


    def on_scan(self, data):
        assert len(data) == 180
        super().on_scan(data)

        delete_in_view = [artf for artf in self.objects_in_view if self.objects_in_view[artf]['expiration'] < self.sim_time]
        for artf in delete_in_view:
            del self.objects_in_view[artf]
            if artf == "rover":
                if (self.debug):
                    print(self.sim_time, self.robot_name, "Expiring rover view and resetting distance to 15m")
                self.rover_distance = 15

        if not self.finish_visually:
            return

        # TODO: avoid obstacles when following (too tight turns)

        if 'rover' in self.objects_in_view.keys() and not self.arrived_message_sent and not self.inException:
            # TODO: Use rover_distance instead?
            # TODO: sometimes we are near rover but don't see its distance
            # other times, we see a rover and its distance but stuck behind a rock
            if self.excavator_yaw is None: # within requested distance and no outstanding yaw, report
                if (
                        self.driving_mode == "approach" and self.scan_distance_to_obstacle/1000.0 < self.target_excavator_distance or
                        self.driving_mode != "approach" and abs(self.rover_distance - self.target_excavator_distance) < 0.100
                ):
                    self.publish("desired_movement", [0, 0, 0])
                    print(self.sim_time, self.robot_name, "Sending arrived message to excavator")
                    self.send_request('external_command excavator_1 arrived')
                    self.arrived_message_sent = True
                    self.excavator_yaw = None
                    self.scan_driving = False
                    if self.debug:
                        print(self.sim_time, self.robot_name, "Arrival handling: distance %.1f, yaw: %.2f, rover angle: %.2f; stopping" % (self.rover_distance, self.yaw, self.rover_angle))

            if self.scan_driving or (abs(self.rover_distance - self.target_excavator_distance) < 0.1 and self.excavator_yaw is not None): # within 1m from target, start adjusting angles
                self.scan_driving = True
                diff = normalizeAnglePIPI(self.yaw - self.excavator_yaw)
                if (self.debug):
                    print(self.sim_time, self.robot_name, "Arrival handling: yaw difference: (%.2f - %.2f)= %.2f, rover angle: %.2f" % (self.yaw, self.excavator_yaw, diff, self.rover_angle))

                if abs(diff) > 0.15: # 10deg, don't bother otherwise
                    if (self.debug):
                        print(self.sim_time, self.robot_name, "Arrival handling: moving on a curve")
                    self.publish("desired_movement", [-8, -9000, math.copysign(0.5 * self.default_effort_level, diff)])
                    #TODO: keep doing until diff=0; however we are bound to overshoot so maybe leave as is
#                elif abs(self.rover_angle) > 0.2:
#                    if (self.debug):
#                        print(self.sim_time, self.robot_name, "Arrival handling: moving sideways")
#                    self.publish("desired_movement", [GO_STRAIGHT, -9000, -math.copysign(0.5 * self.default_effort_level, self.rover_angle)])
#                    #TODO: keep doing until angle=0
                else:
                    self.publish("desired_movement", [0, 0, 0])
                    self.target_excavator_distance = EXCAVATOR_DIGGING_GAP
                    self.excavator_yaw = None
                    self.scan_driving = False

#            else:
#                if (self.debug):
#                    print(self.sim_time, self.robot_name, "Arrival handling: going forward")
#                self.publish("desired_movement", [GO_STRAIGHT, 0, self.default_effort_level])



#        self.rover_angle = rover_center_angle(data)

# vim: expandtab sw=4 ts=4
