"""
  Space Robotics Challenge 2
"""
from scipy import spatial
import numpy as np
import math
from datetime import timedelta

from osgar.bus import BusShutdownException

from moon.controller import SpaceRoboticsChallenge, VirtualBumperException, LidarCollisionException, LidarCollisionMonitor
from osgar.lib.virtual_bumper import VirtualBumper
from osgar.lib.quaternion import euler_zyx

DIG_SAMPLE_COUNT = 12
DIG_GOOD_LOCATION_MASS = 10

class SpaceRoboticsChallengeExcavatorRound2(SpaceRoboticsChallenge):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register("bucket_dig", "bucket_drop")
        self.volatile_dug_up = None

    def on_bucket_info(self, timestamp, bucket_status):
        if bucket_status[1] != 100:
            self.volatile_dug_up = bucket_status[2]

    def run(self):

        def pose_distance(pose1, pose2):
            return math.hypot(pose1[0] - pose2[0], pose1[1] - pose2[1])


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


            while len(vol_list) > 0:

                distance, index = spatial.KDTree(vol_list).query([rx,ry])
                print ("Dist: %f, index: %d" % (distance, index))

                angle = math.atan2(vol_list[index][1] - ry, vol_list[index][0] - rx)
                angle_diff = angle - yaw

                # set up future robot position for the next volatile
                # TODO: adjust based on actual direction of volatile scooped
                rx = vol_list[index][0]
                ry = vol_list[index][1]
                yaw = angle
                vol_list.pop(index)

                try:
                    self.virtual_bumper = VirtualBumper(timedelta(seconds=20), 0.1)

                    # move bucket to the back so that it does not interfere with lidar
                    # TODO: find better position/move for driving
                    self.publish("bucket_drop", math.pi)

                    turn_angle = angle_diff % (2*math.pi)
                    if turn_angle > math.pi:
                        turn_angle = -(2*math.pi - turn_angle)
                    self.turn(turn_angle, timeout=timedelta(seconds=30))
                    # -2.5: target the volatile to be in front of the vehicle
                    # hauler may also push excavator forward a little bit too

                    # split trip in two parts to give hauler a little time to get closer
                    # in order to then follow in line in case the volatile is real close
                    self.go_straight(3, timeout=timedelta(seconds=30))
                    self.wait(timedelta(seconds=5))


                    starting_pose = self.last_position
                    desired_travel_distance = distance - 2.5 - 3
                    try:
                        with LidarCollisionMonitor(self):
                            self.go_straight(desired_travel_distance, timeout=timedelta(seconds=30))
                    except (VirtualBumperException, LidarCollisionException) as e:
                        # TODO: exception in exception (eg steep hill) will just go to the next volatile
                        distance_covered = pose_distance(starting_pose, self.last_position)
                        self.go_straight(-1.0, timeout=timedelta(seconds=20))
                        self.turn(math.radians(90), timeout=timedelta(seconds=10))
                        self.go_straight(5.0, timeout=timedelta(seconds=20))
                        self.turn(math.radians(-90), timeout=timedelta(seconds=10))
                        self.go_straight(6.0, timeout=timedelta(seconds=20))
                        self.turn(math.radians(-90), timeout=timedelta(seconds=10))
                        self.go_straight(5.0, timeout=timedelta(seconds=20))
                        self.turn(math.radians(90), timeout=timedelta(seconds=10))

                        self.go_straight(desired_travel_distance - distance_covered - 5, timeout=timedelta(seconds=20))

                    print ("---- NEXT VOLATILE ----")
                    #self.wait(timedelta(seconds=10))
                    #continue

                    self.set_brakes(True)

                    found_index = None
                    for i in range(DIG_SAMPLE_COUNT):
                        self.volatile_dug_up = None
                        self.publish("bucket_dig", (-math.pi / 2 + i * 2*math.pi / DIG_SAMPLE_COUNT) % (2*math.pi))
                        self.wait(timedelta(seconds=26))
                        # if scooped less than DIG_GOOD_LOCATION_MASS, we can probably do better digging on different angle
                        # still dump first though
                        if self.volatile_dug_up is not None:
                            self.publish("bucket_drop", math.pi)
                            self.wait(timedelta(seconds=28))
                            if self.volatile_dug_up > DIG_GOOD_LOCATION_MASS:
                                found_index = i
                                break
                    if found_index is not None:
                        while self.volatile_dug_up > 0:
                            self.volatile_dug_up = 0
                            self.publish("bucket_dig", (-math.pi / 2 + found_index * 2*math.pi / DIG_SAMPLE_COUNT) % (2*math.pi))
                            self.wait(timedelta(seconds=26))
                            self.publish("bucket_drop", math.pi)
                            self.wait(timedelta(seconds=28))
                    self.set_brakes(False)
                    # move away from hauler to be able to turn
                    # TODO: adjust driving accordingly
                    self.go_straight(1.0, timeout=timedelta(seconds=20))

                except VirtualBumperException:
                    pass


        except BusShutdownException:
            pass

# vim: expandtab sw=4 ts=4
