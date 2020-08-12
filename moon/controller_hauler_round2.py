"""
  Space Robotics Challenge 2
"""

from scipy import spatial
import numpy as np
import math
from datetime import timedelta

from osgar.bus import BusShutdownException

from moon.controller import ps, SpaceRoboticsChallenge, ChangeDriverException, VirtualBumperException, min_dist, LidarCollisionException, LidarCollisionMonitor
from osgar.lib.quaternion import euler_zyx
from osgar.lib.virtual_bumper import VirtualBumper
from osgar.lib.mathex import normalizeAnglePIPI

CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480

TURN_ON = 12 # radius of circle when turning
GO_STRAIGHT = float("inf")
EXCAVATOR_DRIVING_GAP = 1500
EXCAVATOR_DIGGING_GAP = 800

def rover_center_angle(laser_data):
    # positive to the left, negative to the right
    if len(laser_data) > 0:
        # remove ultra near reflections and unlimited values == 0
        laser_data = [x if x > 10 else 20000 for x in laser_data]
        min_index = max_index = None
        for i in range(40, len(laser_data)-40):
            if laser_data[i] < 4000:
                min_index = i
                break
        for i in range(len(laser_data)-40, 40, -1):
            if laser_data[i-1] < 4000:
                max_index = i-1
                break
        if min_index is None or max_index is None:
            return 0.0

        center_index = (min_index + max_index) // 2
        mid_point = len(laser_data) // 2
        rad_direction = -1.3 * (mid_point - center_index) / 50.0
        return rad_direction
    return 0.0


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
        self.target_excavator_distance = EXCAVATOR_DRIVING_GAP
        self.objects_in_view = {}
        self.bin_content = None


    def on_osgar_broadcast(self, data):
        def vslam_reset_time(response):
            self.vslam_reset_at = self.sim_time

        message_target = data.split(" ")[0]
        if message_target == self.robot_name:
            print(self.sim_time, self.robot_name, "Received external_command: %s" % str(data))
            command = data.split(" ")[1]
            if command == "goto":
                # x, y, heading
                self.goto = [float(n) for n in data.split(" ")[2:5]]
                self.send_request('vslam_reset', vslam_reset_time)
            elif command == "approach":
                self.arrived_message_sent = False
                self.target_excavator_distance = EXCAVATOR_DIGGING_GAP
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
            self.set_light_intensity("0.3")

            self.set_brakes(False)

            while not self.true_pose:
                self.wait(timedelta(seconds=1))

            self.virtual_bumper = VirtualBumper(timedelta(seconds=20), 0.1)

            # 3 modes: 1) going to location 2) following rover visually 3) waiting for instructions to go to location

            while True:


                if self.goto is not None:
                    try:
                        with LidarCollisionMonitor(self, 2000):
                            angle_diff = self.get_angle_diff(self.goto, 1)
                            self.turn(angle_diff)
                            self.go_to_location(self.goto, self.default_effort_level, full_turn=False, timeout=timedelta(minutes=2))
                            self.turn(normalizeAnglePIPI(self.goto[2] - self.yaw))
                        self.goto = None
                        self.finish_visually = True
                        self.set_cam_angle(-0.15)


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
                    self.wait(timedelta(seconds=1))
                except ChangeDriverException as e:
                    print(self.sim_time, self.robot_name, "Driver changed to go to location")
                except VirtualBumperException as e: # if bumper while following (e.g, blocked by a rock)
                    self.send_speed_cmd(0.0, 0.0)
                    print(self.sim_time, self.robot_name, "Bumper")
                    self.inException = True
                    self.go_straight(-1) # go 1m in opposite direction
                    self.drive_around_rock(6) # assume 6m the most needed
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

        if self.sim_time is None:
            return

        self.objects_in_view[artifact_type] = {
            "expiration": self.sim_time + timedelta(milliseconds=200)
        }

        if artifact_type == "rover" and not self.arrived_message_sent:
            self.last_rover_timestamp = self.sim_time

            direction = 1 if self.target_excavator_distance < self.scan_distance_to_obstacle else -1

            # if hauler is within 1m of something (presumably excavator), regardless of target distance from excavator, move sideways to center
            if self.scan_distance_to_obstacle < 1000 and abs(self.rover_angle) > 0.2:
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
                if center_x < CAMERA_WIDTH/4:
                    self.publish("desired_movement", [0, 0, self.default_effort_level])
                elif center_x < (CAMERA_WIDTH/2 - 20):
                    self.publish("desired_movement", [TURN_ON, 0, direction * self.default_effort_level])
                elif center_x > 3*CAMERA_WIDTH/4:
                    self.publish("desired_movement", [0, 0, -self.default_effort_level])
                elif center_x > (CAMERA_WIDTH/2 + 20):
                    self.publish("desired_movement", [-TURN_ON, 0, direction * self.default_effort_level])
                else:
                    self.publish("desired_movement", [GO_STRAIGHT, 0, direction * self.default_effort_level])


    def on_scan(self, data):
        assert len(data) == 180
        super().on_scan(data)

        delete_in_view = [artf for artf in self.objects_in_view if self.objects_in_view[artf]['expiration'] < self.sim_time]
        for artf in delete_in_view:
            del self.objects_in_view[artf]

        if not self.finish_visually:
            return

        # TODO: avoid obstacles when following (too tight turns)

        if 'rover' in self.objects_in_view.keys() and self.target_excavator_distance - 100 < self.scan_distance_to_obstacle < self.target_excavator_distance + 100:
            if not self.arrived_message_sent:
                print(self.sim_time, self.robot_name, "Sending arrived message to excavator")
                self.send_request('external_command excavator_1 arrived')
                self.arrived_message_sent = True

            self.publish("desired_movement", [0, 0, 0])

        self.rover_angle = rover_center_angle(data)

# vim: expandtab sw=4 ts=4
