"""
  Space Robotics Challenge 2
"""
from scipy import spatial
import numpy as np
import math
from datetime import timedelta
import traceback

from osgar.bus import BusShutdownException

from moon.controller import distance, SpaceRoboticsChallenge, VirtualBumperException, LidarCollisionException, LidarCollisionMonitor
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
        self.vslam_reset_at = None
        self.use_gimbal = False


    def on_bucket_info(self, bucket_status):
        self.volatile_dug_up = bucket_status

    def on_joint_position(self, data):
        super().on_joint_position(data)
        self.mount_angle = data[self.joint_name.index(b'mount_joint')]

    def on_vslam_pose(self, data):
        super().on_vslam_pose(data)

        if self.sim_time is None or self.last_position is None or self.yaw is None:
            return

        if self.vslam_reset_at is not None and self.sim_time - self.vslam_reset_at > timedelta(seconds=3) and not math.isnan(data[0][0]) and self.tf['vslam']['trans_matrix'] is None:
            # request origin and start tracking in correct coordinates as soon as first mapping lock occurs
            # TODO: another pose may arrive while this request is still being processed (not a big deal, just a ROS error message)
            self.send_request('request_origin', self.register_origin)


    def run(self):

        def process_volatiles(vol_string):
            nonlocal vol_list

            print (self.sim_time, "main-excavator-round2: Volatiles: %s" % vol_string)
            vol_list_one = vol_string.split(',')
            vol_list = [list(map(float, s.split())) for s in vol_list_one]

        def vslam_reset_time(response):
            self.vslam_reset_at = self.sim_time

        vol_list = None

        try:
            self.wait_for_init()

            self.set_cam_angle(-0.1)
            self.set_light_intensity("0.5")

            self.send_request('get_volatile_locations', process_volatiles)

            # move bucket to the back so that it does not interfere with lidar
            # TODO: find better position/move for driving
            # wait for interface to wake up before trying to move arm
            self.publish("bucket_drop", [math.pi, 'reset'])

            try:
                self.go_straight(-3)
            except:
                pass

            self.send_request('vslam_reset', vslam_reset_time)

            while vol_list is None or not self.true_pose:
                self.wait(timedelta(seconds=1))


            while len(vol_list) > 0:
                dist, ind = spatial.KDTree(vol_list).query(self.xyz[0:2], k=len(vol_list))

                for i in range(len(dist)):
                    if abs(self.get_angle_diff(vol_list[ind[i]])) > math.pi/2:
                        print (self.sim_time, "excavator: Distance distance: %f, index: %d in opposite direction, skipping" % (dist[i], ind[i]))
                        continue

                    try:
                        print(self.sim_time, "excavator: Pursuing distance: %f, index: %d" % (dist[i], ind[i]))
                        self.virtual_bumper = VirtualBumper(timedelta(seconds=3), 0.2) # radius of "stuck" area; a little more as the robot flexes
                        with LidarCollisionMonitor(self, 2500): # some distance needed not to lose tracking when seeing only obstacle up front
                            #angle_diff = self.get_angle_diff(vol_list[index],-1) # 1 means get angle for going forward
                            #self.turn(angle_diff, timeout=timedelta(seconds=15))
                            self.go_to_location(vol_list[ind[i]], self.default_effort_level, offset=-1, full_turn=True)

                        print ("---- NEXT VOLATILE ----")
                        #self.wait(timedelta(seconds=10))
                        #vol_list.pop(index)
                        self.set_brakes(True)
                        #self.wait(timedelta(seconds=3))
                        #self.set_brakes(False)
                        #continue


                        found_angle = None
                        for i in range(DIG_SAMPLE_COUNT):
                            self.publish("bucket_dig", [(-math.pi / 2 + i * 2*math.pi / DIG_SAMPLE_COUNT) % (2*math.pi), 'append'])

                        sample_start = self.sim_time
                        while found_angle is None and self.sim_time - sample_start < timedelta(seconds=60):
                            # give sampler 60 seconds to find the volatile, otherwise time out
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

                        if found_angle is not None:
                            scoop_all(found_angle)

                        self.set_brakes(False)

                        vol_list.pop(ind[i])# once scooping finished or given up on, remove volatile from list

                        # move away from hauler to be able to turn
                        # TODO: adjust driving accordingly

                    except LidarCollisionException as e: #TODO: long follow of obstacle causes loss, go along under steeper angle
                        print(self.sim_time, "Lidar")
                        if distance(self.xyz, vol_list[ind[i]]) < 1:
                            continue
                        self.inException = True
                        self.lidar_drive_around()
                        self.inException = False
                    except VirtualBumperException as e:
                        self.send_speed_cmd(0.0, 0.0)
                        print(self.sim_time, "Bumper")
                        if distance(self.xyz, vol_list[ind[i]]) < 1:
                            continue
                        self.inException = True
                        self.go_straight(-1) # go 1m in opposite direction
                        self.turn(self.get_angle_diff(vol_list[ind[i]]), timeout=timedelta(seconds=15))
                        self.drive_around_rock(6) # assume 6m the most needed
                        self.inException = False

                    # when completed or exception, recalculate nearest appropriate volatile
                    break

        except BusShutdownException:
            pass

        print ("EXCAVATOR END")

# vim: expandtab sw=4 ts=4
