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
        self.robot_name = "excavator_1"
        self.hauler_ready = False

    def on_osgar_broadcast(self, data):
        print(self.sim_time, self.robot_name, "Received external_command: %s" % str(data))
        message_target = data.split(" ")[0]
        if message_target == self.robot_name:
            command = data.split(" ")[1]
            if command == "arrived":
                self.hauler_ready = True

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

            print (self.sim_time, self.robot_name, "main-excavator-round2: Volatiles: %s" % vol_string)
            vol_list_one = vol_string.split(',')
            vol_list = [list(map(float, s.split())) for s in vol_list_one]

        def vslam_reset_time(response):
            self.vslam_reset_at = self.sim_time

        vol_list = None

        try:
            self.wait_for_init()

            self.set_brakes(False)

            self.set_cam_angle(0.0)
            self.set_light_intensity("0.1")

            self.send_request('get_volatile_locations', process_volatiles)

            # move bucket to the back so that it does not interfere with lidar
            # TODO: is there a better position to stash the bucket for driving?

            # wait for interface to wake up before trying to move arm
            self.publish("bucket_drop", [math.pi, 'reset'])

            self.send_request('vslam_reset', vslam_reset_time)

            while vol_list is None or not self.true_pose:
                self.wait(timedelta(seconds=1))

            # calculate position behind the excavator to send hauler to, hauler will continue visually from there
            v = np.asmatrix(np.asarray([self.xyz[0], self.xyz[1], 1]))
            c, s = np.cos(self.yaw), np.sin(self.yaw)
            R = np.asmatrix(np.array(((c, -s, 0), (s, c, 0), (0, 0, 1))))
            T = np.asmatrix(np.array(((1, 0, -6),(0, 1, 0),(0, 0, 1))))
            pos =  np.dot(R, np.dot(T, np.dot(R.I, v.T)))
            self.send_request('external_command hauler_1 goto %.1f %.1f %.2f' % (pos[0], pos[1], self.yaw))

            # wait for hauler to start following, will receive osgar broadcast when done
            while not self.hauler_ready:
                self.wait(timedelta(seconds=1))

            while len(vol_list) > 0:
                dist, ind = spatial.KDTree(vol_list).query([0,0], k=len(vol_list))

                for i in range(len(dist)):
                    if abs(self.get_angle_diff(vol_list[ind[i]])) > 3*math.pi/4 and dist[i] < 10:
                        print (self.sim_time, self.robot_name, "%d: excavator: Distance distance: %f, index: %d in opposite direction, skipping" % (i, dist[i], ind[i]))
                        continue
                    if -3*math.pi/4 < math.atan2(vol_list[ind[i]][1] - self.xyz[1], vol_list[ind[i]][0] - self.xyz[0]) < -math.pi/4:
                        print (self.sim_time, self.robot_name, "%d: Too much away from the sun, shadow interference" % i)
                        continue

#                    if distance([0,0], vol_list[ind[i]]) > 35:
#                        print (self.sim_time, self.robot_name, "%d: Too far from center" % i)
#                        continue


                    try:
                        print(self.sim_time, self.robot_name, "excavator: Pursuing volatile index: %d at distance: %f" % (ind[i], dist[i]))
                        self.virtual_bumper = VirtualBumper(timedelta(seconds=3), 0.2) # radius of "stuck" area; a little more as the robot flexes
                        with LidarCollisionMonitor(self, 1500): # some distance needed not to lose tracking when seeing only obstacle up front
                            # turning in place probably not desirable because hauler behind may be in the way, may need to send it away first
                            #angle_diff = self.get_angle_diff(vol_list[ind[i]])
                            #self.turn(angle_diff, timeout=timedelta(seconds=15))
                            self.go_to_location(vol_list[ind[i]], self.default_effort_level, offset=-2, full_turn=True) # extra offset for sliding
                            break

                    except LidarCollisionException as e: #TODO: long follow of obstacle causes loss, go along under steeper angle
                        print(self.sim_time, self.robot_name, "Lidar")
                        if distance(self.xyz, vol_list[ind[i]]) < 1:
                            break
                        self.inException = True
                        self.lidar_drive_around()
                        self.inException = False
                    except VirtualBumperException as e:
                        self.send_speed_cmd(0.0, 0.0)
                        print(self.sim_time, self.robot_name, "Bumper")
                        if distance(self.xyz, vol_list[ind[i]]) < 1:
                            break
                        self.inException = True
                        self.go_straight(-1) # go 1m in opposite direction
                        self.turn(self.get_angle_diff(vol_list[ind[i]]), timeout=timedelta(seconds=15))
                        self.drive_around_rock(6) # assume 6m the most needed
                        self.inException = False

                if i == len(dist) - 1:
                    print("***************** VOLATILES EXHAUSTED ****************************")
                    break

                self.send_speed_cmd(0.0, 0.0)

                print ("---- VOLATILE REACHED ----")
                #self.wait(timedelta(seconds=10))
                #vol_list.pop(index)
                self.set_brakes(True)
                #self.wait(timedelta(seconds=3))
                #self.set_brakes(False)
                #continue


                found_angle = None
                angular_sample_increment = 3*math.pi/2 / DIG_SAMPLE_COUNT # will be sampling 270 degrees (excepting 90deg behind the rover)
                for j in range(DIG_SAMPLE_COUNT // 2):
                    self.publish("bucket_dig", [-j * angular_sample_increment, 'append'])
                for j in range(1, DIG_SAMPLE_COUNT // 2):
                    self.publish("bucket_dig", [j * angular_sample_increment, 'append'])
                self.publish("bucket_drop", [math.pi, 'append'])

                sample_start = self.sim_time
                while found_angle is None and self.sim_time - sample_start < timedelta(seconds=140):
                    # TODO instead of/in addition to timeout, trigger exit by bucket reaching reset position
                    self.wait(timedelta(milliseconds=300))
                    if self.volatile_dug_up[1] != 100:
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

                self.publish("bucket_drop", [math.pi, 'reset'])
                self.set_brakes(False)

                vol_list.pop(ind[i])# once scooping finished or given up on, remove volatile from list


        except BusShutdownException:
            pass

        print ("EXCAVATOR END")

# vim: expandtab sw=4 ts=4
