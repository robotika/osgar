"""
  Space Robotics Challenge 2
"""

from scipy import spatial
import numpy as np
import math
from datetime import timedelta

from osgar.bus import BusShutdownException

from moon.controller import SpaceRoboticsChallenge, ChangeDriverException, VirtualBumperException, min_dist, LidarCollisionException, LidarCollisionMonitor
from osgar.lib.quaternion import euler_zyx
from osgar.lib.virtual_bumper import VirtualBumper

CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480

TURN_ON = 10 # radius of circle when turning
GO_STRAIGHT = float("inf")

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


        self.tracking_excavator = False
        self.straight_ahead_distance = None
        self.approach_distance_timestamp = None
        self.approaching = False
        self.last_rover_timestamp = False
        self.rover_angle = None
        self.use_gimbal = False
        self.vslam_reset_at = None
        self.goto = None

        self.last_excavator_pose = None


    def run(self):

        def vslam_reset_time(response):
            self.vslam_reset_at = self.sim_time



        try:
            self.wait_for_init()
            #self.wait(timedelta(seconds=5))
            self.set_light_intensity("0.5")

            self.send_request('vslam_reset', vslam_reset_time)

            self.virtual_bumper = VirtualBumper(timedelta(seconds=20), 0.1)

            while True:
                try:
                    if self.goto is not None:
                        self.go_to_location(self.goto, self.default_effort_level, offset=-1, full_turn=True)
                        self.goto = None
                    else:
                        while True:
                            self.wait(timedelta(seconds=1))
                except ChangeDriverException as e:
                    print("Driver changed to go to location")
                except LidarCollisionException as e: #TODO: long follow of obstacle causes loss, go along under steeper angle
                    print(self.sim_time, "Lidar")
                    self.inException = True
                    self.lidar_drive_around()
                    self.inException = False
                except VirtualBumperException as e:
                    self.send_speed_cmd(0.0, 0.0)
                    print(self.sim_time, "Bumper")
                    self.inException = True
                    self.go_straight(-1) # go 1m in opposite direction
                    self.drive_around_rock(6) # assume 6m the most needed
                    self.inException = False

        except BusShutdownException:
            pass

        print("HAULER END")

    def on_osgar_broadcast(self, data):
        print("Received external_command: %s" % str(data))
        message_target = data.split(" ")[0]
        if message_target == "hauler_1":
            command = data.split(" ")[1]
            if command == "goto":
                self.goto = [float(n) for n in data.split(" ")[2:4]]
                raise ChangeDriverException


    def on_vslam_pose(self, data):
        super().on_vslam_pose(data)

        if self.sim_time is None or self.last_position is None or self.yaw is None:
            return

        if self.vslam_reset_at is not None and self.sim_time - self.vslam_reset_at > timedelta(seconds=3) and not math.isnan(data[0][0]) and self.tf['vslam']['trans_matrix'] is None:
            # request origin and start tracking in correct coordinates as soon as first mapping lock occurs
            # TODO: another pose may arrive while this request is still being processed (not a big deal, just a ROS error message)
            self.send_request('request_origin', self.register_origin)


    def on_artf(self, data):
        return
        # vol_type, x, y, w, h
        # coordinates are pixels of bounding box
        artifact_type = data[0]
        center_x = data[1] + data[3] / 2
        center_y = data[2] + data[4] / 2
        bbox_size = (data[3] + data[4]) / 2 # calculate avegage in case of substantially non square matches
        img_x, img_y, img_w, img_h = data[1:5]
        nr_of_black = data[5]

        if self.sim_time is None:
            return

        if self.straight_ahead_distance is None:
            return

        if artifact_type == "rover":
            self.last_rover_timestamp = self.sim_time

            if not self.tracking_excavator and self.straight_ahead_distance < 8:
                self.tracking_excavator = True
                raise ChangeDriverException

            if self.tracking_excavator and not self.brakes_on:
                if self.approach_distance_timestamp is not None and self.sim_time - self.approach_distance_timestamp > timedelta(seconds=15):
                    # if was in approach bracket more than X secs, approach
                    # TODO: may be following moving robot within the approach distance envelope for more than 15 secs
                    # then it would give up control before lining up behind a digging robot
                    self.approaching = True
                    self.publish("desired_movement", [GO_STRAIGHT, 0, self.default_effort_level])

                if self.approaching:
                    if self.straight_ahead_distance < 0.3:
                        self.set_brakes(True)

                else:
                    # if too close, move back no matter what
                    if self.straight_ahead_distance < 0.5:
                        self.publish("desired_movement", [GO_STRAIGHT, 0, -self.default_effort_level])

                    if self.straight_ahead_distance < 1.5:
                        if self.rover_angle > 0.2:
                            self.publish("desired_movement", [0, 0, self.default_effort_level])
                        elif self.rover_angle < -0.2:
                            self.publish("desired_movement", [0, 0, -self.default_effort_level])
                        else:
                            # centered and between 0.5 and 2m distant
                            self.publish("desired_movement", [0, 0, 0])

                    else:
                        # if bbox center in left or right quarter, turn in place
                        # else if bbox off center turn while moving
                        # else (bbox in center) go straight
                        if center_x < CAMERA_WIDTH/4:
                            self.publish("desired_movement", [0, 0, self.default_effort_level])
                        elif center_x < (CAMERA_WIDTH/2 - 20):
                            self.publish("desired_movement", [TURN_ON, 0, self.default_effort_level])
                        elif center_x > 3*CAMERA_WIDTH/4:
                            self.publish("desired_movement", [0, 0, -self.default_effort_level])
                        elif center_x > (CAMERA_WIDTH/2 + 20):
                            self.publish("desired_movement", [-TURN_ON, 0, self.default_effort_level])
                        else:
                            self.publish("desired_movement", [GO_STRAIGHT, 0, self.default_effort_level])


    def on_scan(self, data):
        assert len(data) == 180
        super().on_scan(data)
        return

        # if we lose sight of rover for more than X seconds, it means it left and we should release brakes and look for it
        # doesn't work super well because up real close, especially when aligned well, we do not see enough orange to idenfity rover
        # will need robot-to-robot communication for best results
        if (
                self.tracking_excavator and
                self.last_rover_timestamp is not None and
                self.sim_time is not None and
                self.sim_time - self.last_rover_timestamp > timedelta(seconds=30)
        ):
            self.set_brakes(False)
            self.tracking_excavator = False
            raise ChangeDriverException

        midindex = len(data) // 2
        self.straight_ahead_distance = min_dist(data[midindex-20:midindex+20])
#        print(self.straight_ahead_distance)
        # if first time distance in bracket, mark timestamp
        # if leaves bracket, reset to None
        if self.approach_distance_timestamp is None and 0.5 < self.straight_ahead_distance < 2:
            self.approach_distance_timestamp = self.sim_time
        elif 0.5 > self.straight_ahead_distance or self.straight_ahead_distance > 2:
            self.approach_distance_timestamp = None

        self.rover_angle = rover_center_angle(data)

# vim: expandtab sw=4 ts=4
