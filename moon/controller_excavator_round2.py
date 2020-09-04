"""
  Space Robotics Challenge 2
"""
import numpy as np
import math
from datetime import timedelta
import traceback
from functools import cmp_to_key
import cv2

from shapely.geometry import Point, LineString
from shapely.geometry.polygon import Polygon
from statistics import median

from osgar.bus import BusShutdownException

from moon.controller import eulerAnglesToRotationMatrix, translationToMatrix, GO_STRAIGHT, TURN_RADIUS, calc_tangents, pol2cart, ps, distance, SpaceRoboticsChallenge, VirtualBumperException, LidarCollisionException, LidarCollisionMonitor, VSLAMEnabledException, VSLAMDisabledException, VSLAMLostException, VSLAMFoundException
from osgar.lib.virtual_bumper import VirtualBumper
from osgar.lib.quaternion import euler_zyx

from osgar.lib.mathex import normalizeAnglePIPI

DIG_GOOD_LOCATION_MASS = 10
ARRIVAL_OFFSET = -1.5
TYPICAL_VOLATILE_DISTANCE = 2
DISTAL_THRESHOLD = 0.1 # TODO: movement may be too fast to detect when the scooping first happened
PREFERRED_POLYGON = Polygon([(33,18),(38, -5),(29, -25), (6,-44), (-25,-44), (-25,-9),(-32,15),(-9,29)])
SAFE_POLYGON = Polygon([
    (65, 20), (65, -35), (45, -35), (45, -55), (-65, -55), (-65, -42), (-25, -44), (-25, -5), (-65, -5), (-65, 25), (-40, 30), (-36, 55), (-10, 55), (-10, 41.70731707317074), (16, 37.90243902439025), (16, 55), (40, 55), (40, 25), (45.5, 20.5), (65, 20)
],[[(35,13),(45,13),(45,20),(29,36),(-12, 42),(-35,34),(-34, 15),(-14,28),(4, 25), (21, 20)]])
APPROACH_DISTANCE = 2.5

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
        self.distal_angle = None
        self.vol_list = None
        self.vslam_reset_at = None
        self.use_gimbal = False
        self.robot_name = "excavator_1"
        self.hauler_ready = False
        self.vslam_is_enabled = False
        self.hauler_orig_pose = None
        self.full_360_objects = {}
        self.first_distal_angle = None
        self.hauler_pose_requested = False
        self.auto_light_adjustment = False
        self.light_intensity = 0.0
        self.vslam_fail_start = None
        self.vslam_valid = False

    def on_osgar_broadcast(self, data):
        message_target = data.split(" ")[0]
        if message_target == self.robot_name:
            print(self.sim_time, self.robot_name, "Received external_command: %s" % str(data))
            command = data.split(" ")[1]
            if command == "arrived":
                self.hauler_yaw = float(data.split(" ")[2])
                self.hauler_ready = True
            if command == "wait":
                raise WaitRequestedException()
            if command == "resume":
                raise ResumeRequestedException()
            if command == "hauler_true_pose":
                # assumes hauler finished approach as is behind excavator in self.yaw direction
                self.hauler_pose_requested = True
                print(self.sim_time, self.robot_name, "Origin of hauler received: ", str(data.split(" ")[2:]))
                if data.split()[2] == 'origin':
                    origin = [float(x) for x in data.split()[3:]]
                    initial_xyz = origin[:3]
                    initial_quat = origin[3:]
                    initial_rpy = euler_zyx(initial_quat) # note: this is not in roll, pitch, yaw order

                    self.yaw_offset = self.yaw + self.yaw_offset - initial_rpy[0]

                    v = np.asmatrix(np.asarray([initial_xyz[0], initial_xyz[1], 1]))
                    c, s = np.cos(initial_rpy[0]), np.sin(initial_rpy[0])
                    Rr = np.asmatrix(np.array(((c, -s, 0), (s, c, 0), (0, 0, 1))))
                    T = np.asmatrix(np.array(((1, 0, APPROACH_DISTANCE),(0, 1, 0),(0, 0, 1)))) # looking for location in front of origin
                    ex_pos =  np.dot(Rr, np.dot(T, np.dot(Rr.I, v.T)))
                    print(self.sim_time, self.robot_name, "Adjusting XY based on hauler origin: from [%.1f,%.1f] to [%.1f,%.1f]" % (self.xyz[0], self.xyz[1], float(ex_pos[0]), float(ex_pos[1])))

                    if self.tf['vslam']['trans_matrix'] is not None:
                        m = translationToMatrix([float(ex_pos[0])-self.xyz[0], float(ex_pos[1])-self.xyz[1], 0.0])
                        self.tf['vslam']['trans_matrix'] = np.dot(m, self.tf['vslam']['trans_matrix'])
                    else:
                        ex_initial_xyz = [float(ex_pos[0]), float(ex_pos[1]), origin[2]]
                        initial_rpy[0] = self.yaw # keep existing yaw

                        self.xyz = ex_initial_xyz
                        self.xyz_quat = initial_quat

                        for k, obj in self.tf.items():
                        # note: if VSLAM is not tracking at time of register_origin call, the latest reported position will be inaccurate and VSLAM won't work
                            if obj['latest_quat'] is not None:
                                latest_rpy = euler_zyx(obj['latest_quat']) # will be rearranged after offset calculation
                                rpy_offset = [a-b for a,b in zip(initial_rpy, latest_rpy)]
                                rpy_offset.reverse()
                                print(self.sim_time, self.robot_name, "%s RPY offset: %s" % (k, str(rpy_offset)))
                                rot_matrix = np.asmatrix(eulerAnglesToRotationMatrix(rpy_offset)) # TODO: save the original rot matrix

                                xyz_offset = translationToMatrix(obj['latest_xyz'])
                                orig_xyz_offset = translationToMatrix(initial_xyz)

                                obj['trans_matrix'] = np.dot(orig_xyz_offset, np.dot(rot_matrix, xyz_offset.I))


                else:
                    print(self.sim_time, self.robot_name, "Invalid origin format")

            if command == "pose":
                self.hauler_orig_pose = [float(data.split(" ")[2]),float(data.split(" ")[3])]

    def on_left_image(self, data):
        if not self.auto_light_adjustment:
            return
        limg = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
        CAMERA_HEIGHT,CAMERA_WIDTH, _ = limg.shape
        hsv = cv2.cvtColor(limg, cv2.COLOR_BGR2HSV)
        mask = np.zeros((CAMERA_HEIGHT,CAMERA_WIDTH), np.uint8)
        circle_mask = cv2.circle(mask,(CAMERA_HEIGHT//2,CAMERA_WIDTH//2),200,(255,255,255),thickness=-1)
        hist = cv2.calcHist([limg],[2],circle_mask,[256],[0,256])
        topthird = hist[170:]
        brightness = int(sum(topthird) / len(topthird))
        if brightness < 300:
            self.light_intensity = min(1.0, self.light_intensity + 0.1)
            self.send_request('set_light_intensity %s' % str(self.light_intensity))
        elif brightness > 500:
            self.light_intensity = max(0.0, self.light_intensity - 0.1)
            self.send_request('set_light_intensity %s' % str(self.light_intensity))

    def on_vslam_enabled(self, data):
        super().on_vslam_enabled(data)
        if self.vslam_is_enabled != data:
            self.vslam_is_enabled = data

            if not self.true_pose or self.sim_time is None or self.last_position is None or self.yaw is None:
                return

            if self.vslam_is_enabled:
                raise VSLAMEnabledException()
            else:
                raise VSLAMDisabledException()

    def get_extra_status(self):
        if self.volatile_dug_up[1] == 100:
            return "Bucket empty"
        else:
            return ps("Bucket content: Type: %s idx: %d mass: %f" % (self.volatile_dug_up[0], self.volatile_dug_up[1], self.volatile_dug_up[2]))

    def on_bucket_info(self, bucket_status):
        #[0] .. type ('ice'); [1] .. index (100=nothing); [2] .. mass
        self.volatile_dug_up = bucket_status
        if self.first_distal_angle is None and bucket_status[1] != 100:
            self.first_distal_angle = self.distal_angle

    def on_joint_position(self, data):
        super().on_joint_position(data)
        self.mount_angle = data[self.joint_name.index(b'mount_joint')]
        self.distal_angle = data[self.joint_name.index(b'distalarm_joint')]

    def on_vslam_pose(self, data):
        super().on_vslam_pose(data)

        if self.sim_time is None or self.last_position is None or self.yaw is None:
            return

        if math.isnan(data[0][0]) and self.tf['vslam']['trans_matrix'] is not None and not self.hauler_pose_requested: # VSLAM not tracking and potential for re-tracking via hauler
            self.vslam_valid = False
            if self.vslam_fail_start is None:
                self.vslam_fail_start = self.sim_time
            elif self.sim_time - self.vslam_fail_start > timedelta(milliseconds=300):
                self.vslam_fail_start = None
                raise VSLAMLostException()
            return

        self.vslam_fail_start = None
        if not math.isnan(data[0][0]):
            self.vslam_valid = True

        if not self.true_pose and self.vslam_reset_at is not None and self.sim_time - self.vslam_reset_at > timedelta(seconds=3) and not math.isnan(data[0][0]) and self.tf['vslam']['trans_matrix'] is None:
            # request origin and start tracking in correct coordinates as soon as first mapping lock occurs
            # TODO: another pose may arrive while this request is still being processed (not a big deal, just a ROS error message)
            self.send_request('request_origin', self.register_origin)

    def get_next_volatile(self, vol_list):
        # first, remove volatiles that we won't attempt - in unsafe areas, too close or in the direction of sun
        accessible_volatiles = []
        rover = np.asmatrix(np.asarray([self.xyz[0], self.xyz[1], 1]))
        c, s = np.cos(self.yaw), np.sin(self.yaw)
        R = np.asmatrix(np.array(((c, -s, 0), (s, c, 0), (0, 0, 1))))
        TL = np.asmatrix(np.array(((1, 0, 0),(0, 1, -8),(0, 0, 1))))
        TR = np.asmatrix(np.array(((1, 0, 0),(0, 1, 8),(0, 0, 1))))
        pL =  np.dot(R, np.dot(TL, np.dot(R.I, rover.T)))
        pR =  np.dot(R, np.dot(TR, np.dot(R.I, rover.T)))
        turn_center_L = (float(pL[0]), float(pL[1]))
        turn_center_R = (float(pR[0]), float(pR[1]))

        for v in vol_list:

            d = distance(self.xyz, v)
            if not SAFE_POLYGON.contains(Point(v[0],v[1])):
                print (self.sim_time, self.robot_name, "[%.1f,%.1f] not in safe area" % (v[0], v[1]))
                continue
            if (
                    Point(turn_center_L[0],turn_center_L[1]).buffer(TURN_RADIUS+0.1).contains(Point(v[0],v[1])) or
                    Point(turn_center_R[0],turn_center_R[1]).buffer(TURN_RADIUS+0.1).contains(Point(v[0],v[1]))
            ): # within either circle we cannot turn into
                print (self.sim_time, self.robot_name, "[%.1f,%.1f] within turn circles, distance: %f" % (v[0], v[1], d))
                continue
            if -3*math.pi/4 < math.atan2(v[1] - self.xyz[1], v[0] - self.xyz[0]) < -math.pi/4:
                print (self.sim_time, self.robot_name, "[%.1f,%.1f] too much away from the sun, shadow interference" % (v[0], v[1]))
                continue
            accessible_volatiles.append(v)

        # balance distance, angle and length of traversing non-preferred areas
        # want to avoid non-preferred areas the most but if expose similar, choose volatile with shorter travel distance
        # we are modelling an initial turn along a circle followed by straight drive to the destination
        ranked_list = []

        approx = 16
        arc_len = math.pi/approx
        arc_dist = 2 * math.pi * TURN_RADIUS / approx

        # destination point outside turn circle per above test
        for v in accessible_volatiles:
            path_to_vol = []
            c_angular_difference = self.get_angle_diff(v)
            c_distance = distance(v, self.xyz)
            turn_center = turn_center_L if c_angular_difference < 0 else turn_center_R
            tangs = calc_tangents(turn_center, TURN_RADIUS, v)
            tang1_v = np.asmatrix(np.asarray([tangs[0][0], tangs[0][1], 1]))
            tang2_v = np.asmatrix(np.asarray([tangs[1][0], tangs[1][1], 1]))

            i = 1;
            while True:
                phi = -math.copysign(math.pi/2, c_angular_difference) + self.yaw + math.copysign(i*arc_len, c_angular_difference)
                i += 1
                x,y = pol2cart(TURN_RADIUS, phi)
                px = turn_center[0] + x
                py = turn_center[1] + y
                yw = phi - math.pi/2
                path_to_vol.append((px, py))

                rover_t =  np.asmatrix(np.array(((1, 0, px),(0, 1, py),(0, 0, 1))))
                c, s = np.cos(yw), np.sin(yw)
                rover_r = np.asmatrix(np.array(((c, -s, 0), (s, c, 0), (0, 0, 1))))
                mat = np.dot(rover_r, rover_t.I)
                m1 = np.dot(mat, tang1_v.T)
                m2 = np.dot(mat, tang2_v.T)

                if (
                        (distance([m1[0,0], m1[1,0]], [0,0]) < arc_dist+0.1 and m1[0,0]>=-0.1) or
                        (distance([m2[0,0], m2[1,0]], [0,0]) < arc_dist+0.1 and m2[0,0]>=-0.10)
                ):
                    break
            path_to_vol.append((v[0],v[1]))
            ls = LineString(path_to_vol).buffer(1)
            c_nonpreferred_overlap =  ls.difference(PREFERRED_POLYGON).area

            ranked_list.append([v, c_angular_difference, c_nonpreferred_overlap, c_distance])

        # ranked_list: [volatile, angle, area, distance]
        def cmp(a,b):
            return a[3]-b[3] if abs(a[2]-b[2]) < 10 else a[2]-b[2]

        ranked_list = sorted(ranked_list, key=cmp_to_key(cmp)) # if non safe overlap similar, focus on distance
        print (self.sim_time, self.robot_name, "List of volatiles in the order of suitability for traveling to: ", ranked_list)
        return ranked_list[0][0]



    def on_artf(self, data):
        artifact_type = data[0]

        # can't use this before init and don't need once true pose has been set up
        if self.sim_time is None or self.true_pose:
            return

        # NOTE: counting starts before turning, ie if hauler is visible in the beginning, it keeps getting hits
        if artifact_type not in self.full_360_objects.keys():
            self.full_360_objects[artifact_type] = []
        self.full_360_objects[artifact_type].append(self.yaw)

    def run(self):

        def process_volatiles(vol_string):
            nonlocal vol_list

            vol_list_one = vol_string.split(',')
            vol_list = [list(map(float, s.split())) for s in vol_list_one]
            print (self.sim_time, self.robot_name, "Volatiles (%d): " % len(vol_list), vol_list)

        def vslam_reset_time(response):
            self.vslam_reset_at = self.sim_time

        vol_list = None

        try:
            self.wait_for_init()

            self.set_brakes(True) # prevent sliding down while waiting for arm to align

            self.set_cam_angle(-0.05)
            self.set_light_intensity("0.2")

            self.send_request('get_volatile_locations', process_volatiles)

            # move bucket to the back so that it does not interfere with lidar
            # TODO: is there a better position to stash the bucket for driving?

            # wait for interface to wake up before trying to move arm
            self.publish("bucket_drop", [math.pi, 'reset'])

            while self.mount_angle is None or abs(normalizeAnglePIPI(math.pi - self.mount_angle)) > 0.2:
                try:
                    self.wait(timedelta(seconds=1))
                except BusShutdownException:
                    raise
                except Exception as e:
                    print(self.sim_time, self.robot_name, "Exception while waiting for arm to align: ", str(e))


            self.set_brakes(False)

            # first, turn away from hauler
            # turn 360deg until seen, mark angle where the bbox was the biggest, then turn 180deg from there (want to face away as much as possible)
            # then, when hauler turns towards us (and follows?), it should have the same yaw which we can then share with it

            while True:
                try:
                    self.turn(math.radians(30), ang_speed=0.8*self.max_angular_speed, with_stop=False) # get the turning stared
                    self.full_360_objects = {}
                    self.turn(math.radians(360), ang_speed=0.8*self.max_angular_speed, with_stop=False)

                    sum_sin = sum([math.sin(a) for a in self.full_360_objects["rover"]])
                    sum_cos = sum([math.cos(a) for a in self.full_360_objects["rover"]])
                    mid_angle = math.atan2(sum_sin, sum_cos)
                    print(self.sim_time, self.robot_name, "excavator: hauler angle (internal coordinates) [%.2f]" % (normalizeAnglePIPI(math.pi + mid_angle)))
                    turn_needed = normalizeAnglePIPI(math.pi + mid_angle - self.yaw)
                    # turn a little less to allow for extra turn after the effort was stopped
                    self.turn(turn_needed if turn_needed >= 0 else (turn_needed + 2*math.pi), ang_speed=0.8*self.max_angular_speed)
                    break
                except BusShutdownException:
                    raise
                except Exception as e:
                    print(self.sim_time, self.robot_name, "Exception while performing initial turn: ", str(e))

            self.auto_light_adjustment = True
            # self.set_light_intensity("0.4")

            # this will also trigger true pose once slam starts sending data
            self.send_request('vslam_reset', vslam_reset_time)

            while vol_list is None or not self.true_pose:
                try:
                    self.wait(timedelta(seconds=1))
                except BusShutdownException:
                    raise
                except Exception as e:
                    print(self.sim_time, self.robot_name, "Exception while waiting for true pose: ", str(e))

            # calculate position behind the excavator to send hauler to, hauler will continue visually from there
            v = np.asmatrix(np.asarray([self.xyz[0], self.xyz[1], 1]))
            c, s = np.cos(self.yaw), np.sin(self.yaw)
            R = np.asmatrix(np.array(((c, -s, 0), (s, c, 0), (0, 0, 1))))
            T = np.asmatrix(np.array(((1, 0, -8),(0, 1, 0),(0, 0, 1))))
            pos =  np.dot(R, np.dot(T, np.dot(R.I, v.T)))
            #self.send_request('external_command hauler_1 goto %.1f %.1f %.2f' % (pos[0], pos[1], self.yaw))

            #self.send_request('external_command hauler_1 align %f' % self.yaw) # TODO: due to skidding, this yaw may still change

            self.send_request('external_command hauler_1 set_yaw %f' % self.yaw) # tell hauler looking at rover is in 'yaw' direction

            # wait for hauler to send pose
            #while self.hauler_orig_pose is None:
            #    try:
            #        self.wait(timedelta(seconds=1))
            #    except BusShutdownException:
            #        raise
            #    except:
            #        pass

            #dir_angle = math.atan2(self.xyz[1] - self.hauler_orig_pose[1], self.xyz[0] - self.hauler_orig_pose[0])
            #self.send_request('external_command hauler_1 turnto %f' % dir_angle)

            # finish the turn no matter what
            #while True:
            #    try:
            #        self.turn(normalizeAnglePIPI(dir_angle - self.yaw), timeout=timedelta(seconds=15))
            #        break
            #    except BusShutdownException:
            #        raise
            #    except:
            #        pass

            # wait for hauler to start following, will receive osgar broadcast when done
            while not self.hauler_ready:
                try:
                    self.wait(timedelta(seconds=1))
                except BusShutdownException:
                    raise
                except Exception as e:
                    print(self.sim_time, self.robot_name, "Exception while waiting for hauler to arrive: ", str(e))

            self.hauler_ready = False
            self.send_request('external_command hauler_1 approach %f' % self.yaw)

            self.set_brakes(True)
            while not self.hauler_ready:
                try:
                    self.wait(timedelta(seconds=3))
                except BusShutdownException:
                    raise
                except Exception as e:
                    print(self.sim_time, self.robot_name, "Exception while waiting for hauler to arrive: ", str(e))

            self.hauler_ready = False
            self.set_brakes(False)

            self.send_request('external_command hauler_1 follow')
            #self.send_request('external_command hauler_1 set_yaw %f' % self.yaw) # tell hauler looking at rover is in 'yaw' direction

            # reset again in case hauler was driving in front of excavator CANT RESET IF WE MOVED (TURNED)
            #self.vslam_reset_at = None
            #self.send_request('vslam_reset', vslam_reset_time)
            #while self.vslam_reset_at is None or self.sim_time - self.vslam_reset_at < timedelta(seconds=3):
            #    try:
            #        self.wait(timedelta(seconds=1))
            #    except BusShutdownException:
            #        raise
            #    except:
            #        pass

            self.virtual_bumper = VirtualBumper(timedelta(seconds=3), 0.2) # radius of "stuck" area; a little more as the robot flexes


            def pursue_volatile(v):
                goto_path = [v]
                if not SAFE_POLYGON.contains(LineString([v, self.xyz[:2]])):
                    goto_path = [-10,-10] + goto_path
                wait_for_mapping = False
                wait_for_hauler_requested = None
                ARRIVAL_TOLERANCE = 1 # if we are within 1m of target, stop
                while True:
                    try:

                        if not self.vslam_valid: # vslam_valid will only be turned to False if hauler origin hasn't been requested yet
                            self.publish("desired_movement", [0,0,0])

                            # hauler approach, VSLAM should be running
                            self.send_request('external_command hauler_1 approach %f' % self.yaw)
                            self.set_brakes(True)

                            # reset VSLAM (this should reset self.vslam_valid)
                            self.tf['vslam']['trans_matrix'] = None
                            self.vslam_reset_at = None
                            self.send_request('vslam_reset', vslam_reset_time)
                            while self.vslam_reset_at is None or self.sim_time - self.vslam_reset_at < timedelta(seconds=3):
                                try:
                                    self.wait(timedelta(seconds=1))
                                except BusShutdownException:
                                    raise
                                except Exception as e:
                                    print(self.sim_time, self.robot_name, "Exception while waiting for VSLAM reset: ", str(e))


                            while not self.hauler_ready:
                                try:
                                    self.wait(timedelta(seconds=3))
                                except BusShutdownException:
                                    raise
                                except Exception as e:
                                    print(self.sim_time, self.robot_name, "Exception while waiting for hauler to be ready: ", str(e))

                            self.hauler_ready = False

                            self.send_request('external_command hauler_1 request_true_pose')
                            try:
                                self.wait(timedelta(seconds=5)) # wait for pose to update (TODO: wait for a flag instead)
                            except BusShutdownException:
                                raise
                            except Exception as e:
                                print(self.sim_time, self.robot_name, "Exception while waiting for true pose: ", str(e))


                            self.set_brakes(False)
                            self.send_request('external_command hauler_1 follow')



                        print(self.sim_time, self.robot_name, "excavator: Pursuing volatile [%.1f,%.1f]" % (v[0],v[1]))
                        while wait_for_mapping or (wait_for_hauler_requested is not None and self.sim_time < wait_for_hauler_requested):
                            self.wait(timedelta(seconds=1))

                        with LidarCollisionMonitor(self, 1500): # some distance needed not to lose tracking when seeing only obstacle up front
                            # turning in place probably not desirable because hauler behind may be in the way, may need to send it away first
                            if distance(self.xyz, v) > ARRIVAL_TOLERANCE: # only turn if we are further than 2m, if closer, it probably means there was an exception while we were already on location
                                angle_diff = self.get_angle_diff(v)
                                #self.turn(math.copysign(min(abs(angle_diff),math.pi/4),angle_diff), timeout=timedelta(seconds=15))
                            # ideal position is 0.5m in front of the excavator
                            # TODO: fine-tune offset and tolerance given slipping
                            while len(goto_path) > 1:
                                self.go_to_location(goto_path[0], self.default_effort_level, tolerance=8.0, full_turn=True) # extra offset for sliding
                                goto_path.pop(0)
                            # extra offset for sliding?
                            if len(goto_path) == 1:
                                self.go_to_location(goto_path[0], self.default_effort_level, offset=ARRIVAL_OFFSET, full_turn=True, timeout=timedelta(minutes=5), tolerance=ARRIVAL_TOLERANCE)
                                goto_path.pop(0)

                            return v #arrived

                    except LidarCollisionException as e: #TODO: long follow of obstacle causes loss, go along under steeper angle
                        print(self.sim_time, self.robot_name, "Lidar")
                        if distance(self.xyz, v) < 1:
                            return v #arrived
                        self.inException = True
                        try:
                            self.lidar_drive_around()
                        except ResumeRequestedException as e:
                            wait_for_hauler_requested = self.sim_time + timedelta(seconds=5) # resume only after 5 seconds to give hauler chance to get closer
                            print(self.sim_time, self.robot_name, "Hauler wants to resume")
                        except WaitRequestedException as e:
                            self.send_speed_cmd(0.0, 0.0)
                            wait_for_hauler_requested = self.sim_time + timedelta(minutes=2) # will wait at most 2 minutes
                            print(self.sim_time, self.robot_name, "Hauler requested to wait")
                        except VSLAMDisabledException as e:
                            print(self.sim_time, self.robot_name, "VSLAM: mapping disabled, waiting")
                            self.send_speed_cmd(0.0, 0.0)
                            wait_for_mapping = True
                        except VSLAMEnabledException as e:
                            print(self.sim_time, self.robot_name, "VSLAM: mapping re-enabled")
                            wait_for_mapping = False
                        except VSLAMLostException as e:
                            print(self.sim_time, self.robot_name, "VSLAM lost, re-localize via hauler")

                        self.inException = False
                    except VirtualBumperException as e:
                        self.send_speed_cmd(0.0, 0.0)
                        print(self.sim_time, self.robot_name, "Bumper")
                        if distance(self.xyz, v) < 1:
                            return v # arrived
                        self.inException = True
                        try:
                            self.go_straight(-1) # go 1m in opposite direction
                            angle_diff = self.get_angle_diff(v)
                            self.drive_around_rock(-math.copysign(5, angle_diff)) # assume 6m the most needed
                        except ResumeRequestedException as e:
                            wait_for_hauler_requested = self.sim_time + timedelta(seconds=5)
                            print(self.sim_time, self.robot_name, "Hauler wants to resume")
                        except WaitRequestedException as e:
                            self.send_speed_cmd(0.0, 0.0)
                            wait_for_hauler_requested = self.sim_time + timedelta(minutes=2)
                            print(self.sim_time, self.robot_name, "Hauler requested to wait")
                        except VSLAMDisabledException as e:
                            print(self.sim_time, self.robot_name, "VSLAM: mapping disabled, waiting")
                            self.send_speed_cmd(0.0, 0.0)
                            wait_for_mapping = True
                        except VSLAMEnabledException as e:
                            print(self.sim_time, self.robot_name, "VSLAM: mapping re-enabled")
                            wait_for_mapping = False
                        except VSLAMLostException as e:
                            print(self.sim_time, self.robot_name, "VSLAM lost, re-localize via hauler")

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
                        wait_for_hauler_requested = self.sim_time + timedelta(minutes=2)
                        print(self.sim_time, self.robot_name, "Hauler requested to wait")
                    except ResumeRequestedException as e:
                        wait_for_hauler_requested = self.sim_time + timedelta(seconds=5)
                        print(self.sim_time, self.robot_name, "Hauler wants to resume")
                    except VSLAMLostException as e:
                        print(self.sim_time, self.robot_name, "VSLAM lost, re-localize via hauler")



            def reposition(mount_angle, distal_angle):
                print(self.sim_time, self.robot_name, "Adjusting digging position: mount angle: %.2f, distal angle: %.2f" % (normalizeAnglePIPI(mount_angle), distal_angle))
                # if found an angle of a volatile but it's too far, come closer before digging
                # if distal arm was negative during volatile detection, volatile is away from rover; otherwise it is under the rover
                self.publish("bucket_drop", [math.pi, 'reset']) # first, stash arm and wait for it so that visual navigation doesn't confuse hauler
                try:
                    self.wait(timedelta(seconds=10)) # TODO: wait for angles to match
                except BusShutdownException:
                    raise
                except Exception as e:
                    print(self.sim_time, self.robot_name, "Exception while waiting 10s for arm to reset: ", str(e))


                self.send_request('external_command hauler_1 follow')
                try:
                    self.wait(timedelta(seconds=3))
                except BusShutdownException:
                    raise
                except Exception as e:
                    print(self.sim_time, self.robot_name, "Exception while waiting for hauler to arrive: ", str(e))


                start_pose = self.xyz
                start_time = self.sim_time
                effort = self.default_effort_level/2 if distal_angle < DISTAL_THRESHOLD else -self.default_effort_level/2
                if abs(normalizeAnglePIPI(mount_angle)) < math.pi/2:
                    self.publish("desired_movement", [GO_STRAIGHT, int(100*math.degrees(normalizeAnglePIPI(mount_angle))), effort])
                else:
                    self.publish("desired_movement", [GO_STRAIGHT, -int(100*math.degrees(normalizeAnglePIPI(math.pi - mount_angle))), effort])

                while distance(start_pose, self.xyz) < 0.7: # go 0.7m closer to volatile
                    try:
                        self.wait(timedelta(milliseconds=100))
                    except BusShutdownException:
                        raise
                    except Exception as e:
                        print(self.sim_time, self.robot_name, "Exception while adjusting position: ", str(e))

                self.publish("desired_movement", [0,0,0])
                try:
                    self.wait(timedelta(seconds=3))
                except BusShutdownException:
                    raise
                except Exception as e:
                    print(self.sim_time, self.robot_name, "Exception while waiting to come to stop: ", str(e))

                #self.send_request('external_command hauler_1 approach %f' % normalizeAnglePIPI(self.yaw + mount_angle))

                #self.hauler_ready = False
                #while not self.hauler_ready:
                #    try:
                #        self.wait(timedelta(seconds=1))
                #    except BusShutdownException:
                #        raise
                #    except:
                #        pass
            # end of reposition function

            while len(vol_list) > 0:
                v = self.get_next_volatile(vol_list)
                if v is None:
                    break
                pursue_volatile(v)

                accum = 0 # to check if we picked up 100.0 total and can finish
                for attempt_offset in [0.0, +4.0, -8.0]:
                    try:
                        self.go_straight(attempt_offset)
                    except BusShutdownException:
                        raise
                    except Exception as e:
                        # excess pitch or other exception could happen but we are stopped so should stabilize
                        print(self.sim_time, self.robot_name, "Exception when adjusting linearly", str(e))

                    self.send_speed_cmd(0.0, 0.0)

                    print ("---- VOLATILE REACHED ----")

                    try:
                        self.wait(timedelta(seconds=2))
                    except BusShutdownException:
                        raise
                    except Exception as e:
                        print(self.sim_time, self.robot_name, "Exception while waiting to come to stop: ", str(e))

                    self.set_brakes(True)

                    # TODO: could look for volatile while waiting
                    # wait for hauler to start following, will receive osgar broadcast when done
                    #while not self.hauler_ready:
                    #    try:
                    #        self.wait(timedelta(seconds=1))
                    #    except BusShutdownException:
                    #        raise
                    #    except:
                    #        # excess pitch or other exception could happen but we are stopped so should stabilize
                    #        pass

                    #self.volatile_reached = False # TEST
                    #self.send_request('external_command hauler_1 backout') # Testing
                    #continue # testing

                    self.send_request('external_command hauler_1 onhold') # re-follow after each 0,+2,-2 attempt

                    drop_angle = math.pi # if yaw is just approximated, drop straigt back instead of at hauler yaw direction
                    #drop_angle = normalizeAnglePIPI(math.pi - normalizeAnglePIPI(self.yaw - self.hauler_yaw))
                    found_angle = None

                    # mount 0.4-1.25 or 1.85-2.75 means we can't reach under the vehicle because of the wheels
                    mount_angle_sequence = [0.0, 0.39, -0.39, -0.83, 0.83, 1.26, -1.26, -1.55, 1.55, 1.84, -1.84, -2.2, 2.2, 2.6, -2.6, -2.9, 2.9, 3.14]
                    for a in mount_angle_sequence:
                        self.publish("bucket_dig", [a, 'append'])

                    while found_angle is None:
                        best_mount_angle = None
                        best_angle_volume = 0
                        best_distal_angle = None

                        exit_time = self.sim_time + timedelta(seconds=250)
                        while found_angle is None and self.sim_time < exit_time:
                            # TODO instead of/in addition to timeout, trigger exit by bucket reaching reset position
                            try:
                                self.wait(timedelta(milliseconds=100))
                            except BusShutdownException:
                                raise
                            except Exception as e:
                                print(self.sim_time, self.robot_name, "Exception while waiting in loop: ", str(e))

                            if self.volatile_dug_up[1] != 100:
                                # we found volatile
                                if best_mount_angle is None:
                                    # first time volatile: raise arm, bring hauler to the other side, drop load into bin
                                    #drop_angle = (self.mount_angle + math.pi) % (2*math.pi)

                                    self.publish("bucket_dig", [self.mount_angle, 'standby']) # lift arm and clear rest of queue
                                    self.hauler_ready = False
                                    nma = normalizeAnglePIPI(self.mount_angle)
                                    nma = max(-math.pi/2, min(math.pi/2, nma)) # only move around the back of the robot
                                    self.send_request('external_command hauler_1 approach %f' % normalizeAnglePIPI(self.yaw + nma))

                                    # NOTE: this approach may bump excavator, scoop volume may worsen
                                    while not self.hauler_ready:
                                        try:
                                            self.wait(timedelta(seconds=1))
                                        except BusShutdownException:
                                            raise
                                        except Exception as e:
                                            print(self.sim_time, self.robot_name, "Exception while waiting for hauler to arrive: ", str(e))
                                    self.hauler_ready = False


                                    # wait for hauler to arrive, get its yaw, then we know which direction to drop
                                    drop_angle = normalizeAnglePIPI((self.hauler_yaw - self.yaw) - math.pi)
                                    print(self.sim_time, self.robot_name, "Drop angle set to: %.2f" % drop_angle)

                                    exit_time = self.sim_time + timedelta(seconds=80)
                                    self.publish("bucket_drop", [drop_angle, 'append']) # this will be duplicated below but need to queue now or load would be lost by another dig happening first
                                    self.publish("bucket_dig", [self.mount_angle - 0.3, 'append'])
                                    self.publish("bucket_dig", [self.mount_angle + 0.3, 'append'])
                                    # TODO: this plan doesn't end once these moves are executed, only with timeout

                                if best_mount_angle is None or self.volatile_dug_up[2] > best_angle_volume:
                                    best_distal_angle = self.first_distal_angle
                                    best_angle_volume = self.volatile_dug_up[2]
                                    best_mount_angle = self.mount_angle
                                accum += self.volatile_dug_up[2]
                                if self.volatile_dug_up[2] > DIG_GOOD_LOCATION_MASS:
                                    found_angle = self.mount_angle
                                # go for volatile drop (to hauler), wait until finished
                                self.publish("bucket_drop", [drop_angle, 'prepend'])
                                while self.volatile_dug_up[1] != 100:
                                    try:
                                        self.wait(timedelta(milliseconds=300))
                                    except BusShutdownException:
                                        raise
                                    except Exception as e:
                                        print(self.sim_time, self.robot_name, "Exception while waiting to empty bin: ", str(e))
                                self.first_distal_angle = None


                        if found_angle is None and best_mount_angle is not None:
                            # didn't find good position (timeout) but found something - reposition and try again
                            reposition(best_mount_angle, best_distal_angle)
                            # after repositioning, try again the whole circle starting with best angle first
                            for a in mount_angle_sequence:
                                self.publish("bucket_dig", [normalizeAnglePIPI(best_mount_angle + a), 'append'])
                        else: # found good position or found nothing at all
                            break

                    def scoop_all(angle):
                        nonlocal accum
                        while True:
                            dig_start = self.sim_time
                            self.publish("bucket_dig", [angle, 'reset'])
                            while self.volatile_dug_up[1] == 100:
                                try:
                                    self.wait(timedelta(milliseconds=300))
                                except BusShutdownException:
                                    raise
                                except Exception as e:
                                    print(self.sim_time, self.robot_name, "Exception while waiting to detect volatile: ", str(e))
                                if self.sim_time - dig_start > timedelta(seconds=120): # TODO: timeout needs to be adjusted to the ultimate digging plan
                                    # move bucket out of the way and continue to next volatile
                                    if self.debug:
                                        print(self.sim_time, self.robot_name, "Arm movement timed out or nothing to dig, terminating scooping")
                                    self.publish("bucket_drop", [drop_angle, 'append'])
                                    return False
                            accum += self.volatile_dug_up[2]
                            self.publish("bucket_drop", [drop_angle, 'append'])
                            while self.volatile_dug_up[1] != 100:
                                try:
                                    self.wait(timedelta(milliseconds=300))
                                except BusShutdownException:
                                    raise
                                except Exception as e:
                                    print(self.sim_time, self.robot_name, "Exception while waiting to empty bin: ", str(e))

                            if abs(accum - 100.0) < 0.0001:
                                print(self.sim_time, self.robot_name, "Volatile amount %.2f transfered, terminating" % accum)
                                # TODO: adjust transformation matrix using real location of the volatile
                                # TODO: assumes 100 to be total volatile at location, actual total reported initially, parse and match instead
                                return True

                    full_success = False
                    if found_angle is not None:
                        # have best position, dig everything up, adjust position, move to next volatile
                        if scoop_all(found_angle):
                            vol_list.remove(v)# once scooping finished or given up on, remove volatile from list

                            v = np.asmatrix(np.asarray([v[0], v[1], 1]))
                            c, s = np.cos(self.yaw), np.sin(self.yaw)
                            Rr = np.asmatrix(np.array(((c, -s, 0), (s, c, 0), (0, 0, 1))))

                            # now in coord system WRT robot
                            T = np.asmatrix(np.array(((1, 0, -TYPICAL_VOLATILE_DISTANCE),(0, 1, 0),(0, 0, 1))))
                            c, s = np.cos(best_mount_angle), np.sin(best_mount_angle)
                            Rv = np.asmatrix(np.array(((c, -s, 0), (s, c, 0), (0, 0, 1))))

                            true_pos =  np.dot(Rr, np.dot(Rv, np.dot(T, np.dot(Rv.I, np.dot(Rr.I, v.T)))))

                            m = translationToMatrix([float(true_pos[0])-self.xyz[0], float(true_pos[1])-self.xyz[1], 0.0])
                            print(self.sim_time, self.robot_name, "Adjusting XY based on volatile location: from [%.1f,%.1f] to [%.1f,%.1f]" % (self.xyz[0], self.xyz[1], float(true_pos[0]), float(true_pos[1])))
                            self.tf['vslam']['trans_matrix'] = np.dot(m, self.tf['vslam']['trans_matrix'])
                            full_success = True


                    # wait for bucket to be dropped and/or position reset before moving on
                    self.publish("bucket_drop", [math.pi, 'reset'])
                    while self.volatile_dug_up[1] != 100 or (self.mount_angle is None or abs(normalizeAnglePIPI(math.pi - self.mount_angle)) > 0.2):
                        try:
                            self.wait(timedelta(seconds=1))
                        except BusShutdownException:
                            raise
                        except Exception as e:
                            print(self.sim_time, self.robot_name, "Exception while waiting to empty bin: ", str(e))

                    self.set_brakes(False)
                    if full_success:
                        break # do not de-attempt 2m back and 2m front

                    # otherwise re-follow after giving up on this position
                    self.send_request('external_command hauler_1 follow')
                # end of for(0,+2,-4)

                if accum == 0 and not self.hauler_pose_requested:
                    self.set_brakes(True)

                    self.send_request('external_command hauler_1 approach %f' % self.yaw)
                    while not self.hauler_ready:
                        try:
                            self.wait(timedelta(seconds=1))
                        except BusShutdownException:
                            raise
                        except Exception as e:
                            print(self.sim_time, self.robot_name, "Exception while waiting for hauler to arrive: ", str(e))

                    self.hauler_ready = False

                    self.send_request('external_command hauler_1 request_true_pose')
                    try:
                        self.wait(timedelta(seconds=5)) # wait for pose to update (TODO: wait for a flag instead)
                    except BusShutdownException:
                        raise
                    except Exception as e:
                        print(self.sim_time, self.robot_name, "Exception while waiting to true pose: ", str(e))

                    self.set_brakes(False)

                self.send_request('external_command hauler_1 follow') # re-follow after each volatile


            print("***************** VOLATILES EXHAUSTED ****************************")


        except BusShutdownException:
            pass

        print ("EXCAVATOR END")

# vim: expandtab sw=4 ts=4
