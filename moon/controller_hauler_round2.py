"""
  Space Robotics Challenge 2
"""

from scipy import spatial
import numpy as np
import math
from datetime import timedelta

from osgar.bus import BusShutdownException

from moon.controller import SpaceRoboticsChallenge, ChangeDriverException, VirtualBumperException, min_dist
from osgar.lib.quaternion import euler_zyx
from osgar.lib.virtual_bumper import VirtualBumper

CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480

SPEED_ON = 10 # only +/0/- matters
TURN_ON = 10 # radius of circle when turning
GO_STRAIGHT = float("inf")

class SpaceRoboticsChallengeHaulerRound2(SpaceRoboticsChallenge):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register("desired_movement")


        self.tracking_excavator = False
        self.straight_ahead_distance = None
        self.approach_distance_timestamp = None
        self.approaching = False
        self.last_rover_timestamp = False

    def run(self):

        try:
            print('Wait for definition of last_position and yaw')
            while self.last_position is None or self.yaw is None:
                self.update()  # define self.time
            print('done at', self.time)

            self.virtual_bumper = VirtualBumper(timedelta(seconds=20), 0.1)

            while True:
                try:
                    print("Turning")
                    self.turn(math.radians(360), timeout=timedelta(seconds=30))
                except ChangeDriverException as e:
                    print("Driver changed: %s" % str(e))
                except VirtualBumperException:
                    pass

                while True:
                    try:
                        self.wait(timedelta(seconds=10))
                    except VirtualBumperException:
                        pass
                    except ChangeDriverException as e:
                        print("Driver changed: %s" % str(e))
                        break

        except BusShutdownException:
            pass

    def on_artf(self, timestamp, data):
        # vol_type, x, y, w, h
        # coordinates are pixels of bounding box
        artifact_type = data[0]
        center_x = data[1] + data[3] / 2
        center_y = data[2] + data[4] / 2
        bbox_size = (data[3] + data[4]) / 2 # calculate avegage in case of substantially non square matches
        img_x, img_y, img_w, img_h = data[1:5]
        nr_of_black = data[4]

        if artifact_type == "rover":
            self.last_rover_timestamp = timestamp

            if not self.tracking_excavator and (CAMERA_WIDTH/2 - 20 < center_x < CAMERA_WIDTH/2 + 20) and self.straight_ahead_distance < 4:
                self.tracking_excavator = True
                raise ChangeDriverException


            if self.approach_distance_timestamp is not None and self.time - self.approach_distance_timestamp > timedelta(seconds=15):
                # if was in approach bracket more than 5 secs, approach
                self.approaching = True
                self.publish("desired_movement", [GO_STRAIGHT, 0, SPEED_ON])

            if self.approaching:
                if self.straight_ahead_distance < 0.3 and not self.brakes_on:
                    self.set_brakes(True)

            else:
                if self.straight_ahead_distance > 2: # if centered, keep going straight
                    if center_x < (CAMERA_WIDTH/2 - 20): # if homebase to the left, steer left
                        self.publish("desired_movement", [TURN_ON, 0, SPEED_ON])
                    elif center_x > (CAMERA_WIDTH/2 + 20):
                        self.publish("desired_movement", [-TURN_ON, 0, SPEED_ON])
                    else:
                        self.publish("desired_movement", [GO_STRAIGHT, 0, SPEED_ON])
                        if self.brakes_on:
                            self.approaching = False
                            self.set_brakes(False)
                elif self.straight_ahead_distance < 0.5:
                    self.publish("desired_movement", [GO_STRAIGHT, 0, -SPEED_ON])
                else:
                    if center_x < (CAMERA_WIDTH/2 - 20): # if homebase to the left, steer left
                        self.publish("desired_movement", [0, 0, SPEED_ON])
                    elif center_x > (CAMERA_WIDTH/2 + 20):
                        self.publish("desired_movement", [0, 0, -SPEED_ON])
                    else:
                        self.publish("desired_movement", [0, 0, 0])


    def on_scan(self, timestamp, data):
        assert len(data) == 180
        super().on_scan(timestamp, data)

        midindex = len(data) // 2
        self.straight_ahead_distance = min_dist(data[midindex-40:midindex+40])
#        print(self.straight_ahead_distance)
        # if first time distance in bracket, mark timestamp
        # if leaves bracket, reset to None
        if self.approach_distance_timestamp is None and 0.5 < self.straight_ahead_distance < 2:
            self.approach_distance_timestamp = timestamp
        elif 0.5 > self.straight_ahead_distance or self.straight_ahead_distance > 2:
            self.approach_distance_timestamp = None



# vim: expandtab sw=4 ts=4
