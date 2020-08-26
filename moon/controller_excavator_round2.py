"""
  Space Robotics Challenge 2
"""
from scipy import spatial
import numpy as np
import math
from datetime import timedelta
import traceback

from shapely.geometry import Point, LineString
from shapely.geometry.polygon import Polygon

from osgar.bus import BusShutdownException

from moon.controller import ps, distance, SpaceRoboticsChallenge, VirtualBumperException, LidarCollisionException, LidarCollisionMonitor, VSLAMEnabledException, VSLAMDisabledException
from osgar.lib.virtual_bumper import VirtualBumper
from osgar.lib.quaternion import euler_zyx
from osgar.lib.mathex import normalizeAnglePIPI

DIG_GOOD_LOCATION_MASS = 10

PREFERRED_POLYGON = Polygon([(33,18),(42, -5),(29, -25), (6,-44), (-25,-33), (-25,-9),(-32,15),(-9,29)])
SAFE_POLYGON = Polygon([
    (65,20),(65,-35),(45,-35),(45,-55),(-65,-55),(-65,-42),(-25,-42),(-25,-5),(-65,-5),(-65,25),(-40,30),(-36,55),(-10,55),(-10,31),(16,31),(16,55),(40,55),(40,20)
])

class WaitRequestedException(Exception):
    pass
class ResumeRequestedException(Exception):
    pass

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
        self.volatile_reached = False
        self.vslam_is_enabled = False
        self.hauler_orig_pose = None

    def on_osgar_broadcast(self, data):
        message_target = data.split(" ")[0]
        if message_target == self.robot_name:
            print(self.sim_time, self.robot_name, "Received external_command: %s" % str(data))
            command = data.split(" ")[1]
            if command == "arrived":
                self.hauler_yaw = float(data.split(" ")[2])
                self.hauler_ready = True
            if command == "wait":
                raise WaitRequestedException
            if command == "resume":
                raise ResumeRequestedException
            if command == "pose":
                self.hauler_orig_pose = [float(data.split(" ")[2]),float(data.split(" ")[3])]


    def on_vslam_enabled(self, data):
        super().on_vslam_enabled(data)
        if self.vslam_is_enabled != data:
            self.vslam_is_enabled = data

            if not self.true_pose or self.sim_time is None or self.last_position is None or self.yaw is None:
                return

            if self.vslam_is_enabled:
                raise VSLAMEnabledException
            else:
                raise VSLAMDisabledException

    def get_extra_status(self):
        if self.volatile_dug_up[1] == 100:
            return "Bucket empty"
        else:
            return ps("Bucket content: Type: %s idx: %d mass: %f" % (self.volatile_dug_up[0], self.volatile_dug_up[1], self.volatile_dug_up[2]))

    def on_bucket_info(self, bucket_status):
        #[0] .. type ('ice'); [1] .. index (100=nothing); [2] .. mass
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

            self.set_cam_angle(-0.05)
            self.set_light_intensity("0.4")

            self.send_request('get_volatile_locations', process_volatiles)

            # move bucket to the back so that it does not interfere with lidar
            # TODO: is there a better position to stash the bucket for driving?

            # wait for interface to wake up before trying to move arm
            self.publish("bucket_drop", [math.pi, 'reset'])

            while self.mount_angle is None or abs(normalizeAnglePIPI(math.pi - self.mount_angle)) > 0.2:
                try:
                    self.wait(timedelta(seconds=1))
                except:
                    pass

            self.send_request('vslam_reset', vslam_reset_time)

            while vol_list is None or not self.true_pose:
                try:
                    self.wait(timedelta(seconds=1))
                except:
                    pass

            # calculate position behind the excavator to send hauler to, hauler will continue visually from there
            v = np.asmatrix(np.asarray([self.xyz[0], self.xyz[1], 1]))
            c, s = np.cos(self.yaw), np.sin(self.yaw)
            R = np.asmatrix(np.array(((c, -s, 0), (s, c, 0), (0, 0, 1))))
            T = np.asmatrix(np.array(((1, 0, -8),(0, 1, 0),(0, 0, 1))))
            pos =  np.dot(R, np.dot(T, np.dot(R.I, v.T)))
            #self.send_request('external_command hauler_1 goto %.1f %.1f %.2f' % (pos[0], pos[1], self.yaw))

            #self.send_request('external_command hauler_1 align %f' % self.yaw) # TODO: due to skidding, this yaw may still change

            # wait for hauler to send pose
            while self.hauler_orig_pose is None:
                try:
                    self.wait(timedelta(seconds=1))
                except:
                    pass

            dir_angle = math.atan2(self.xyz[1] - self.hauler_orig_pose[1], self.xyz[0] - self.hauler_orig_pose[0])
            self.send_request('external_command hauler_1 turnto %f' % dir_angle)

            # finish the turn no matter what
            while True:
                try:
                    self.turn(normalizeAnglePIPI(dir_angle - self.yaw), timeout=timedelta(seconds=15))
                    break
                except:
                    pass

            # wait for hauler to start following, will receive osgar broadcast when done
            while not self.hauler_ready:
                try:
                    self.wait(timedelta(seconds=1))
                except:
                    pass

            self.send_request('external_command hauler_1 follow')

            # reset again in case hauler was driving in front of excavator CANT RESET IF WE MOVED (TURNED)
            # self.vslam_reset_at = None
            # self.send_request('vslam_reset', vslam_reset_time)
            #while self.vslam_reset_at is None or self.sim_time - self.vslam_reset_at < timedelta(seconds=3):
            #    self.wait(timedelta(seconds=1))

            self.virtual_bumper = VirtualBumper(timedelta(seconds=3), 0.2) # radius of "stuck" area; a little more as the robot flexes

            while len(vol_list) > 0:
                dist, ind = spatial.KDTree(vol_list).query(self.xyz[:2], k=len(vol_list))

                def pursue_volatile():
                    nonlocal dist, ind

                    goto_path = []
                    for i in range(len(dist)):
                        d = distance(self.xyz, vol_list[ind[i]])
                        if  d < 5: # too close/just did it
                            print (self.sim_time, self.robot_name, "%d: excavator: Distance: %f, index: %d, skipping" % (i, d, ind[i]))
                            continue

                        if abs(self.get_angle_diff(vol_list[ind[i]])) > 3*math.pi/4 and d < 10:
                            print (self.sim_time, self.robot_name, "%d: excavator: Distance: %f, index: %d in opposite direction, skipping" % (i, d, ind[i]))
                            continue
                        if -3*math.pi/4 < math.atan2(vol_list[ind[i]][1] - self.xyz[1], vol_list[ind[i]][0] - self.xyz[0]) < -math.pi/4:
                            print (self.sim_time, self.robot_name, "%d: Too much away from the sun, shadow interference" % i)
                            continue
                        if not PREFERRED_POLYGON.contains(LineString([
                                (self.xyz[0],self.xyz[1]),
                                (vol_list[ind[i]][0], vol_list[ind[i]][1])
                        ])):
                            print (self.sim_time, self.robot_name, "%d: excavator: Path to index %d not through preferred polygon, skipping" % (i, ind[i]))
                            continue
                        goto_path.append(vol_list[ind[i]])
                        break

                    if len(goto_path) == 0:
                        for i in range(len(dist)):
                            d = distance(self.xyz, vol_list[ind[i]])
                            if  d < 5: # too close/just did it
                                print (self.sim_time, self.robot_name, "%d: excavator: Distance: %f, index: %d, skipping" % (i, d, ind[i]))
                                continue

                            if abs(self.get_angle_diff(vol_list[ind[i]])) > 3*math.pi/4 and d < 10:
                                print (self.sim_time, self.robot_name, "%d: excavator: Distance: %f, index: %d in opposite direction, skipping" % (i, d, ind[i]))
                                continue
                            if -3*math.pi/4 < math.atan2(vol_list[ind[i]][1] - self.xyz[1], vol_list[ind[i]][0] - self.xyz[0]) < -math.pi/4:
                                print (self.sim_time, self.robot_name, "%d: Too much away from the sun, shadow interference" % i)
                                continue
                            if not SAFE_POLYGON.contains(LineString([
                                    (self.xyz[0],self.xyz[1]),
                                    (vol_list[ind[i]][0], vol_list[ind[i]][1])
                            ])):
                                print (self.sim_time, self.robot_name, "%d: excavator: Path to index %d not safe, going through -10,-10, skipping" % (i, ind[i]))
                                goto_path.append([-10,-10])

                            goto_path.append(vol_list[ind[i]])
                            break

                    if len(goto_path) > 0:
                        wait_for_mapping = False
                        wait_for_hauler_requested = False
                        ARRIVAL_TOLERANCE = 1 # if we are within 1m of target, stop
                        while True:
                            try:
                                print(self.sim_time, self.robot_name, "excavator: Pursuing volatile [%.1f,%.1f] at distance: %f" % (vol_list[ind[i]][0],vol_list[ind[i]][1], d))
                                while wait_for_mapping or wait_for_hauler_requested:
                                    self.wait(timedelta(seconds=1))

                                with LidarCollisionMonitor(self, 1500): # some distance needed not to lose tracking when seeing only obstacle up front
                                    # turning in place probably not desirable because hauler behind may be in the way, may need to send it away first
                                    if distance(self.xyz, vol_list[ind[i]]) > ARRIVAL_TOLERANCE: # only turn if we are further than 2m, if closer, it probably means there was an exception while we were already on location
                                        angle_diff = self.get_angle_diff(vol_list[ind[i]])
                                        self.turn(math.copysign(min(abs(angle_diff),math.pi/4),angle_diff), timeout=timedelta(seconds=15))
                                    # ideal position is 0.5m in front of the excavator
                                    # TODO: fine-tune offset and tolerance given slipping
                                    while len(goto_path) > 1:
                                        self.go_to_location(goto_path[0], self.default_effort_level, tolerance=8.0, full_turn=True) # extra offset for sliding
                                        goto_path.pop(0)
                                    # extra offset for sliding?
                                    if len(goto_path) == 1:
                                        self.go_to_location(goto_path[0], self.default_effort_level, offset=-1.5, full_turn=True, timeout=timedelta(minutes=5), tolerance=ARRIVAL_TOLERANCE)
                                        goto_path.pop(0)

                                    self.wait(timedelta(seconds=2)) # wait to come to a stop
                                    self.send_request('external_command hauler_1 approach %f' % self.yaw) # TODO: due to skidding, this yaw may still change after sending
                                    self.hauler_ready = False
                                    return ind[i] #arrived

                            except LidarCollisionException as e: #TODO: long follow of obstacle causes loss, go along under steeper angle
                                print(self.sim_time, self.robot_name, "Lidar")
                                if distance(self.xyz, vol_list[ind[i]]) < 1:
                                    return ind[i] #arrived
                                self.inException = True
                                try:
                                    self.lidar_drive_around()
                                except ResumeRequestedException as e:
                                    wait_for_hauler_requested = False
                                    print(self.sim_time, self.robot_name, "Hauler wants to resume")
                                except WaitRequestedException as e:
                                    self.send_speed_cmd(0.0, 0.0)
                                    wait_for_hauler_requested = True
                                    print(self.sim_time, self.robot_name, "Hauler requested to wait")
                                except VSLAMDisabledException as e:
                                    print(self.sim_time, self.robot_name, "VSLAM: mapping disabled, waiting")
                                    self.send_speed_cmd(0.0, 0.0)
                                    wait_for_mapping = True
                                except VSLAMEnabledException as e:
                                    print(self.sim_time, self.robot_name, "VSLAM: mapping re-enabled")
                                    wait_for_mapping = False

                                self.inException = False
                            except VirtualBumperException as e:
                                self.send_speed_cmd(0.0, 0.0)
                                print(self.sim_time, self.robot_name, "Bumper")
                                if distance(self.xyz, vol_list[ind[i]]) < 1:
                                    return ind[i] # arrived
                                self.inException = True
                                try:
                                    self.go_straight(-1) # go 1m in opposite direction
                                    angle_diff = self.get_angle_diff(vol_list[ind[i]])
                                    self.drive_around_rock(-math.copysign(5, angle_diff)) # assume 6m the most needed
                                except ResumeRequestedException as e:
                                    wait_for_hauler_requested = False
                                    print(self.sim_time, self.robot_name, "Hauler wants to resume")
                                except WaitRequestedException as e:
                                    self.send_speed_cmd(0.0, 0.0)
                                    wait_for_hauler_requested = True
                                    print(self.sim_time, self.robot_name, "Hauler requested to wait")
                                except VSLAMDisabledException as e:
                                    print(self.sim_time, self.robot_name, "VSLAM: mapping disabled, waiting")
                                    self.send_speed_cmd(0.0, 0.0)
                                    wait_for_mapping = True
                                except VSLAMEnabledException as e:
                                    print(self.sim_time, self.robot_name, "VSLAM: mapping re-enabled")
                                    wait_for_mapping = False

                                self.inException = False
                            except VSLAMDisabledException as e:
                                print(self.sim_time, self.robot_name, "VSLAM: mapping disabled, waiting")
                                self.send_speed_cmd(0.0, 0.0)
                                wait_for_mapping = True
                            except VSLAMEnabledException as e:
                                print(self.sim_time, self.robot_name, "VSLAM: mapping re-enabled")
                                wait_for_mapping = False
                            except WaitRequestedException as e:
                                self.send_speed_cmd(0.0, 0.0)
                                wait_for_hauler_requested = True
                                print(self.sim_time, self.robot_name, "Hauler requested to wait")
                            except ResumeRequestedException as e:
                                wait_for_hauler_requested = False
                                print(self.sim_time, self.robot_name, "Hauler wants to resume")
                    else:
                        print("***************** VOLATILES EXHAUSTED ****************************")
                        return None

                picked_up_index = pursue_volatile()
                if picked_up_index is None:
                    return # quitting, maybe we can do more

                self.send_speed_cmd(0.0, 0.0)

                print ("---- VOLATILE REACHED ----")
                #self.wait(timedelta(seconds=10))
                #vol_list.pop(index)
                self.set_brakes(True)
                #self.wait(timedelta(seconds=3))
                #self.set_brakes(False)
                #continue
                self.volatile_reached = True

                # TODO: could look for volatile while waiting
                # wait for hauler to start following, will receive osgar broadcast when done
                while not self.hauler_ready:
                    try:
                        self.wait(timedelta(seconds=1))
                    except:
                        # excess pitch or other exception could happen but we are stopped so should stabilize
                        pass

                #self.volatile_reached = False # TEST
                #self.send_request('external_command hauler_1 backout') # Testing
                #continue # testing


                drop_angle = normalizeAnglePIPI(math.pi - normalizeAnglePIPI(self.yaw - self.hauler_yaw))
                found_angle = None
                best_angle = None
                # mount 0.4-1.25 or 1.85-2.75 means we can't reach under the vehicle because of the wheels
                mount_angle_sequence = [0.0, 0.39, -0.39, -0.83, 0.83, 1.26, -1.26, -1.55, 1.55, 1.84, -1.84, -2.2, 2.2, 2.6, -2.6]
                for a in mount_angle_sequence:
                    self.publish("bucket_dig", [a, 'append'])
                self.publish("bucket_drop", [drop_angle, 'append'])

                sample_start = self.sim_time
                accum = 0 # to check if we picked up 100.0 total and can finish
                while found_angle is None and self.sim_time - sample_start < timedelta(seconds=220):
                    # TODO instead of/in addition to timeout, trigger exit by bucket reaching reset position
                    try:
                        self.wait(timedelta(milliseconds=300))
                    except:
                        pass
                    if self.volatile_dug_up[1] != 100:
                        # we found first volatile

                        if best_angle is None or self.volatile_dug_up[2] > accum:
                            best_angle = self.mount_angle
                        accum += self.volatile_dug_up[2]
                        if self.volatile_dug_up[2] > DIG_GOOD_LOCATION_MASS:
                            found_angle = self.mount_angle
                        # go for volatile drop (to hauler), wait until finished
                        self.publish("bucket_drop", [drop_angle, 'prepend'])
                        while self.volatile_dug_up[1] != 100:
                            try:
                                self.wait(timedelta(milliseconds=300))
                            except:
                                pass

                def scoop_all(angle):
                    nonlocal accum
                    while True:
                        dig_start = self.sim_time
                        self.publish("bucket_dig", [angle, 'reset'])
                        while self.volatile_dug_up[1] == 100:
                            try:
                                self.wait(timedelta(milliseconds=300))
                            except:
                                pass
                            if self.sim_time - dig_start > timedelta(seconds=50): # TODO: timeout needs to be adjusted to the ultimate digging plan
                                # move bucket out of the way and continue to next volatile
                                self.publish("bucket_drop", [drop_angle, 'append'])
                                return False
                        accum += self.volatile_dug_up[2]
                        self.publish("bucket_drop", [drop_angle, 'append'])
                        while self.volatile_dug_up[1] != 100:
                            try:
                                self.wait(timedelta(milliseconds=300))
                            except:
                                pass
                        if abs(accum - 100.0) < 0.0001:
                            print(self.sim_time, self.robot_name, "Volatile amount %.2f transfered, terminating" % accum)
                            # TODO: assumes 100 to be total volatile at location, actual total reported initially, parse and match instead
                            return True

                if found_angle is None and best_angle is not None and accum >= 3.0:
                    # do not attempt to collect all if initial amount too low, would take too long
                    found_angle = best_angle
                if found_angle is not None:
                    if scoop_all(found_angle):
                        vol_list.pop(picked_up_index)# once scooping finished or given up on, remove volatile from list

                self.publish("bucket_drop", [math.pi, 'reset'])

                # wait for bucket to be dropped and/or position reset before moving on
                while self.volatile_dug_up[1] != 100 or (self.mount_angle is None or abs(normalizeAnglePIPI(math.pi - self.mount_angle)) > 0.2):
                    try:
                        self.wait(timedelta(seconds=1))
                    except:
                        pass

                self.send_request('external_command hauler_1 follow')

                self.set_brakes(False)

                self.volatile_reached = False



        except BusShutdownException:
            pass

        print ("EXCAVATOR END")

# vim: expandtab sw=4 ts=4
