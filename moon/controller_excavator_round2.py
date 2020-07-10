"""
  Space Robotics Challenge 2
"""
from scipy import spatial
import numpy as np
import math
from datetime import timedelta
import traceback

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
        self.mount_angle = None
        self.vol_list = None

    def on_bucket_info(self, bucket_status):
        self.volatile_dug_up = bucket_status

    def on_joint_position(self, data):
        super().on_joint_position(data)
        self.mount_angle = data[self.joint_name.index(b'mount_joint')]

    def run(self):

        def pose_distance(pose1, pose2):
            return math.hypot(pose1[0] - pose2[0], pose1[1] - pose2[1])

        def process_volatiles(vol_string):
            nonlocal vol_list

            print (self.sim_time, "main-excavator-round2: Volatiles: %s" % vol_string)
            vol_list_one = vol_string.split(',')
            vol_list = [list(map(float, s.split())) for s in vol_list_one]

        def process_origin(message):
            nonlocal origin_reported, xyz, quat, rx, ry, yaw
            if message.split()[0] == 'origin':
                origin = [float(x) for x in message.split()[1:]]

                xyz = origin[:3]
                quat = origin[3:]

                rx = xyz[0]
                ry = xyz[1]
                yaw = euler_zyx(quat)[0]
            origin_reported = True


        vol_list = None

        origin_reported = False
        xyz = [0,]*3
        quat = [0,0,0,1]
        rx = 10
        ry = 10
        yaw = 0

        self.wait_for_init()

        try:
            self.send_request('get_volatile_locations', process_volatiles)
            self.send_request('request_origin', process_origin)

            while vol_list is None or not origin_reported:
                self.wait(timedelta(seconds=1))

            # move bucket to the back so that it does not interfere with lidar
            # TODO: find better position/move for driving
            # wait for interface to wake up before trying to move arm
            self.publish("bucket_drop", [math.pi, 'reset'])

            while len(vol_list) > 0:

                def make_unprotected_move(step):
                    move, value = step
                    if move == 'go_straight':
                        self.go_straight(value, timeout=timedelta(seconds=abs(value*4)))
                    else:
                        self.turn(value, timeout=timedelta(seconds=10))

                def drive_step(next_step):
                    queue = [next_step]
                    in_exception = False
                    while len(queue) > 0:
                        one_step = queue.pop(0)
                        move, value = one_step
                        if move == "in_exception":
                            in_exception = value
                            continue
                        starting_pose = self.last_position
                        starting_yaw = self.yaw
                        if not in_exception:
                            self.virtual_bumper = VirtualBumper(timedelta(seconds=20), 0.1)
                            try:
                                with LidarCollisionMonitor(self):
                                    make_unprotected_move(one_step)
                            except (VirtualBumperException, LidarCollisionException) as e:
                                in_exception = True
                                self.virtual_bumper = None
                                print(self.sim_time, "excavator_controller exception in %s: %s" % (move, str(e)))
                                traceback.print_exc()
                                recovery_queue = []
                                if move == "go_straight":
                                    distance_covered = pose_distance(starting_pose, self.last_position)
                                    print(self.sim_time, "excavator_controller exception: distance covered so far: %f" % distance_covered)
                                    recovery_queue.append(("turn", math.radians(90)))
                                    recovery_queue.append(("go_straight", -1))
                                    recovery_queue.append(("go_straight", 5.0))
                                    recovery_queue.append(("in_exception", False))
                                    recovery_queue.append(("turn", math.radians(-90)))
                                    recovery_queue.append(("go_straight", 9.0))
                                    recovery_queue.append(("turn", math.radians(-90)))
                                    recovery_queue.append(("go_straight", 5.0))
                                    recovery_queue.append(("turn", math.radians(90)))
                                    recovery_queue.append(("go_straight", value - distance_covered - 8))
                                else:
                                    recovery_queue.append(("go_straight", -1))
                                    recovery_queue.append(("turn", math.radians(value - (self.yaw - starting_yaw))))
                                    recovery_queue.append(("in_exception", False))
                                    recovery_queue.append(("go_straight", 1))
                                queue = recovery_queue + queue

                        else:
                            make_unprotected_move(one_step)

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

                    turn_angle = angle_diff % (2*math.pi)
                    if turn_angle > math.pi:
                        turn_angle = -(2*math.pi - turn_angle)
                    drive_step(("turn", turn_angle))

                    # -2.5: target the volatile to be in front of the vehicle
                    # hauler may also push excavator forward a little bit too
                    desired_travel_distance = distance - 2.5
                    drive_step(("go_straight", desired_travel_distance))

                    print ("---- NEXT VOLATILE ----")
                    #self.wait(timedelta(seconds=10))
                    #continue

                    self.set_brakes(True)

                    found_angle = None
                    for i in range(DIG_SAMPLE_COUNT):
                        self.publish("bucket_dig", [(-math.pi / 2 + i * 2*math.pi / DIG_SAMPLE_COUNT) % (2*math.pi), 'append'])

                    while found_angle is None:
                        # TODO: add timeout, try something else if no volatile found
                        while self.volatile_dug_up[1] == 100:
                            self.wait(timedelta(milliseconds=300))
                        if self.volatile_dug_up[2] > DIG_GOOD_LOCATION_MASS:
                            found_angle = self.mount_angle

                        # go for volatile drop (to hauler), wait until finished
                        self.publish("bucket_drop", [math.pi, 'prepend'])
                        while self.volatile_dug_up[1] != 100:
                            self.wait(timedelta(milliseconds=300))

                    def scoop_all(angle):
                        while True:
                            dig_start = self.sim_time
                            self.publish("bucket_dig", [angle, 'reset'])
                            while self.volatile_dug_up[1] == 100:
                                self.wait(timedelta(milliseconds=300))
                                if self.sim_time - dig_start > timedelta(seconds=20):
                                    # move bucket out of the way and continue to next volatile
                                    self.publish("bucket_drop", [math.pi, 'append'])
                                    return
                            self.publish("bucket_drop", [math.pi, 'append'])
                            while self.volatile_dug_up[1] != 100:
                                self.wait(timedelta(milliseconds=300))

                    scoop_all(found_angle)

                    self.set_brakes(False)
                    # move away from hauler to be able to turn
                    # TODO: adjust driving accordingly
                    drive_step(("go_straight", 0.3))

                except VirtualBumperException:
                    pass


        except BusShutdownException:
            pass

        print ("EXCAVATOR END")

# vim: expandtab sw=4 ts=4
