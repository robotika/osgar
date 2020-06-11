"""
  Space Robotics Challenge 2
"""
from scipy import spatial
import numpy as np
import math
from datetime import timedelta

from osgar.bus import BusShutdownException

from moon.controller import SpaceRoboticsChallenge, VirtualBumperException
from osgar.lib.virtual_bumper import VirtualBumper
from osgar.lib.quaternion import euler_zyx

class SpaceRoboticsChallengeExcavatorRound2(SpaceRoboticsChallenge):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register("dig")
        self.bucket_status = None
        self.volatile_dug_up = False

    def on_bucket_info(self, timestamp, data):
        self.bucket_status = data
        if self.bucket_status[1] != 100:
            self.volatile_dug_up = True

    def run(self):

        try:
            print('Wait for definition of last_position and yaw')
            while self.last_position is None or self.yaw is None:
                self.update()  # define self.time
            print('done at', self.time)

            vol_string = self.send_request('get_volatile_locations').decode('ascii')
            vol_list_one = vol_string.split(',')
            vol_list = [list(map(float, s.split())) for s in vol_list_one]


            message = self.send_request('request_origin').decode("ascii")
            origin = [float(x) for x in message.split()[1:]]

            xyz = origin[:3]
            quat = origin[3:]

            rx = xyz[0]
            ry = xyz[1]
            yaw = euler_zyx(quat)[0]

            distance, index = spatial.KDTree(vol_list).query([rx,ry])

            print ("Dist: %f, index: %d" % (distance, index))

            angle = math.atan2(ry - vol_list[index][1], rx - vol_list[index][0])

            angle_diff = angle - yaw

            try:
                self.virtual_bumper = VirtualBumper(timedelta(seconds=20), 0.1)
                self.turn((angle_diff + math.pi) % (2*math.pi), timeout=timedelta(seconds=30))
                self.go_straight(distance - 2, timeout=timedelta(seconds=30)) # -2: target the volatile to be in front of the vehicle
                self.set_brakes(True)

                while True:
                    for i in range(8):
                        self.volatile_dug_up = False
                        self.publish("dig", [i * 2*math.pi / 8, math.pi])
                        self.wait(timedelta(seconds=55))
                        if not self.volatile_dug_up:
                            continue
                        while self.volatile_dug_up:
                            self.volatile_dug_up = False
                            self.publish("dig", [i * 2*math.pi / 8, math.pi])
                            self.wait(timedelta(seconds=55))
                        return
            except VirtualBumperException:
                pass


        except BusShutdownException:
            pass

# vim: expandtab sw=4 ts=4
