"""
  Space Robotics Challenge 2
"""
import numpy as np
import math
from datetime import timedelta
import traceback
import sys
from functools import cmp_to_key
import cv2

from shapely.geometry import Point, LineString, MultiPolygon, Polygon
from statistics import median

from osgar.bus import BusShutdownException

from moon.controller import eulerAnglesToRotationMatrix, translationToMatrix, GO_STRAIGHT, TURN_RADIUS, calc_tangents, pol2cart, ps, distance, SpaceRoboticsChallenge, MoonException, VirtualBumperException, LidarCollisionException, LidarCollisionMonitor, VSLAMEnabledException, VSLAMDisabledException, VSLAMLostException, VSLAMFoundException
from osgar.lib.virtual_bumper import VirtualBumper
from osgar.lib.quaternion import euler_zyx

from osgar.lib.mathex import normalizeAnglePIPI

DIG_GOOD_LOCATION_MASS = 10
GROUP_ANGLES_MAX_DIFF = 0.3
ARRIVAL_OFFSET = -1.5
TYPICAL_VOLATILE_DISTANCE = 2
DISTAL_THRESHOLD = 0.145 # TODO: movement may be too fast to detect when the scooping first happened
PREFERRED_POLYGON = Polygon([(-32,9),(-25,20),(-16,26),(-6,26),(3.5,22),(14.5,25.5),(23.5,20),(30,9),(35,1),(30,-13.5),(21,-24),(11,-30),(5,-36),(-18,-32),(-19,-17),(-26,-4)])

SAFE_POLYGON = PREFERRED_POLYGON.union(MultiPolygon([
    Polygon([(-30,9),(-43,15),(-41,25),(-28,38),(-15,44.5),(-13,50.5),(-33,51),(-38,36),(-53,26),(-61,3),(-26,-1)]),
    Polygon([(-45,-39),(-58,-40),(-59,-50),(43,-50),(43,-31),(60,-27),(65,30),(51,15),(46,-18), (29,-12.5), (27, -15), (43,-22), (27,-37),(4,-31),(-15,-30)])
    ]))

APPROACH_DISTANCE = 2.5

class WaitRequestedException(MoonException):
    pass
class ResumeRequestedException(MoonException):
    pass
class NewVolatileTargetException(MoonException):
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
        self.full_360_distances = []
        self.first_distal_angle = None
        self.hauler_pose_requested = False
        self.auto_light_adjustment = False
        self.light_intensity = 0.0
        self.vslam_fail_start = None
        self.vslam_valid = False
        self.visual_navigation_enabled = False
        self.lidar_distance_enabled = False
        self.current_volatile = None
        self.going_to_volatile = False
        self.unsafe_location = Polygon([(-100, -100), (-100, 100), (100, 100), (100, -100)]).difference(SAFE_POLYGON)

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
                                orig_xyz_offset = translationToMatrix(ex_initial_xyz)

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
            print (self.sim_time, self.robot_name, "Volatile in bucket at distal: %.2f" % self.distal_angle)
            self.first_distal_angle = self.distal_angle

    def on_joint_position(self, data):
        super().on_joint_position(data)
        self.mount_angle = data[self.joint_name.index(b'mount_joint')]
        self.distal_angle = data[self.joint_name.index(b'distalarm_joint')]

    def on_sim_clock(self, data):
        super().on_sim_clock(data)
        if self.going_to_volatile and data[0] % 5 == 0 and data[1] == 0:
            # if pursuing volatile then every two seconds rerun trajectory optimization and if better volatile than planned ahead, re-plan route
            v = self.get_next_volatile()
            if v != self.current_volatile and distance(self.xyz, self.current_volatile) > 6: # do not change volatile if almost arrived to the current one
                print (self.sim_time, self.robot_name, "New volatile target: [%.1f,%.1f]" % (v[0], v[1]))
                self.current_volatile = v
                raise NewVolatileTargetException()


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

    def get_next_volatile(self):
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

        for v in self.vol_list:

            d = distance(self.xyz, v)
            if not SAFE_POLYGON.contains(Point(v[0],v[1])):
                print (self.sim_time, self.robot_name, "[%.1f,%.1f] not in safe area" % (v[0], v[1]))
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
            turn_center = turn_center_L if c_angular_difference < 0 else turn_center_R
            tangs = calc_tangents(turn_center, TURN_RADIUS, v)
            if tangs is not None:
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

            else:
                # if we had to go with a volatile within the turning radius, do not build a driving arc, just use straight line
                path_to_vol.append((self.xyz[0],self.xyz[1]))
            path_to_vol.append((v[0],v[1]))
            ls = LineString(path_to_vol).buffer(1)
            c_nonpreferred_overlap =  ls.difference(PREFERRED_POLYGON).area

            ranked_list.append([v, c_angular_difference, c_nonpreferred_overlap, ls.area])

        # ranked_list: [volatile, angle, area, distance]
        def cmp(a,b):
            nonlocal turn_center_L, turn_center_R
            #a: [[x,y], angular_diff, non_preferred_overlap, distance_along_driving_path]
            a_v, _, a_overlap, a_distance = a
            b_v, _, b_overlap, b_distance = b


            a_cost = 100 if (
                    Point(turn_center_L[0],turn_center_L[1]).buffer(TURN_RADIUS+0.1).contains(Point(a_v[0],a_v[1])) or
                    Point(turn_center_R[0],turn_center_R[1]).buffer(TURN_RADIUS+0.1).contains(Point(a_v[0],a_v[1]))
            ) else 0
            b_cost = 100 if (
                    Point(turn_center_L[0],turn_center_L[1]).buffer(TURN_RADIUS+0.1).contains(Point(b_v[0],b_v[1])) or
                    Point(turn_center_R[0],turn_center_R[1]).buffer(TURN_RADIUS+0.1).contains(Point(b_v[0],b_v[1]))
            ) else 0

            a_cost += 200 if -3*math.pi/4 < math.atan2(a_v[1] - self.xyz[1], a_v[0] - self.xyz[0]) < -math.pi/4 and a_distance > 15 else 0
            a_cost += 200 if -3*math.pi/4 < math.atan2(a_v[1] - self.xyz[1], a_v[0] - self.xyz[0]) < -math.pi/4 and a_distance > 15 else 0

            a_cost += a_distance
            b_cost += b_distance

            return a_cost - b_cost if abs(a_overlap - b_overlap) < 10 else a_overlap - b_overlap

        ranked_list = sorted(ranked_list, key=cmp_to_key(cmp)) # if non safe overlap similar, focus on distance
        print (self.sim_time, self.robot_name, "List of volatiles in the order of suitability for traveling to: ", ranked_list)
        return ranked_list[0][0]

    def on_scan(self, data):
        super().on_scan(data)
        if self.sim_time is None or self.true_pose:
            return

        if self.lidar_distance_enabled:
            self.full_360_distances.append({'yaw': self.yaw, 'distance': self.scan_distance_to_obstacle})


    def on_artf(self, data):
        artifact_type = data[0]

        # can't use this before init and don't need once true pose has been set up
        if self.sim_time is None or self.true_pose:
            return

        # NOTE: counting starts before turning, ie if hauler is visible in the beginning, it keeps getting hits
        if self.visual_navigation_enabled:
            if artifact_type not in self.full_360_objects.keys():
                self.full_360_objects[artifact_type] = []
            self.full_360_objects[artifact_type].append({'yaw': self.yaw, 'distance': self.scan_distance_to_obstacle})

    def wait_for_hauler(self):
        while not self.hauler_ready:
            try:
                self.wait(timedelta(seconds=1))
            except MoonException as e:
                print(self.sim_time, self.robot_name, "Exception while waiting for hauler to arrive: ", str(e))
        self.hauler_ready = False

    def wait_for_arm(self, angle):
        wait_start = self.sim_time
        while self.sim_time - wait_start < timedelta(seconds=20) and (self.mount_angle is None or abs(normalizeAnglePIPI(angle - self.mount_angle)) > 0.2):
            try:
                self.wait(timedelta(seconds=1))
            except MoonException as e:
                print(self.sim_time, self.robot_name, "Exception while waiting for arm to align: ", str(e))

    def run(self):

        def process_volatiles(vol_string):

            vol_list_one = vol_string.split(',')
            self.vol_list = [list(map(float, s.split())) for s in vol_list_one]
            print (self.sim_time, self.robot_name, "Volatiles (%d): " % len(self.vol_list), self.vol_list)

        def vslam_reset_time(response):
            self.vslam_reset_at = self.sim_time

        try:
            self.wait_for_init()

            self.set_cam_angle(-0.05)
            self.set_light_intensity("0.2")

            self.send_request('get_volatile_locations', process_volatiles)

            # move bucket to the back so that it does not interfere with lidar
            # TODO: is there a better position to stash the bucket for driving?

            # move arm to the back, let robot slide off of a hill if that's where it starts
            self.publish("bucket_drop", [math.pi, 'reset'])

            # point wheels downwards and wait until on flat terrain or timeout
            print(self.sim_time, self.robot_name, "Letting excavator slide down the hill")
            start_drifting = self.sim_time
            while self.sim_time - start_drifting < timedelta(seconds=20) and (abs(self.pitch) > 0.05 or abs(self.roll) > 0.05):
                try:
                    attitude = math.asin(math.sin(self.roll) / math.sqrt(math.sin(self.pitch)**2+math.sin(self.roll)**2))
                    if self.debug:
                        print(self.sim_time, self.robot_name, "Vehicle attitude: %.2f" % attitude)
                    self.publish("desired_movement", [GO_STRAIGHT, math.degrees(attitude * 100), 0.1])
                    self.wait(timedelta(milliseconds=100))
                except MoonException as e:
                    print(self.sim_time, self.robot_name, "Exception while waiting for excavator to settle: ", str(e))
            self.publish("desired_movement", [0, 0, 0])

            self.wait_for_arm(math.pi)

            self.set_brakes(False) # start applying mild braking

            # initial 360 turn to find emptiest area
            print(self.sim_time, self.robot_name, "Scanning 360 degrees of horizon to find widest segment with far enough free space")
            self.lidar_distance_enabled = True
            while True:
                try:
                    self.turn(math.radians(30), ang_speed=0.8*self.max_angular_speed, with_stop=False, timeout=timedelta(seconds=40)) # get the turning stared
                    self.full_360_distances = []
                    self.turn(math.radians(360), ang_speed=0.8*self.max_angular_speed, timeout=timedelta(seconds=40))
                    break
                except MoonException as e:
                    print(self.sim_time, self.robot_name, "Exception while performing initial turn: ", str(e))

            self.lidar_distance_enabled = False
            self.set_cam_angle(-0.05)

            min_distance_obj = min(self.full_360_distances, key=lambda x: x['distance'])
            # if min distance in all directions is more than 5m, no need to drive anywhere
            if min_distance_obj['distance'] < 5000:

                def distance_grouper(iterable):
                    prev = None
                    group = []
                    for item in iterable:
                        if not prev or abs(item['distance'] - prev['distance']) <= 300:
                            group.append(item)
                        else:
                            yield group
                            group = [item]
                        prev = item
                    if group:
                        yield group

                clusters = list(enumerate(distance_grouper(self.full_360_distances)))
                m = []
                for c in clusters:
                    sum_sin = sum([math.sin(a['yaw']) for a in c[1]])
                    sum_cos = sum([math.cos(a['yaw']) for a in c[1]])
                    mid_angle = math.atan2(sum_sin, sum_cos)
                    if median([o['distance'] for o in c[1]]) > 6000:
                        m.append({'yaw': mid_angle, 'wedge_width': len(c[1])})
                if len(m) > 0:
                    bestyawobj = max(m, key=lambda x: x['wedge_width'])

                    try:
                        print(self.sim_time, self.robot_name, "excavator: Driving into-clear-space, yaw (internal coordinates) [%.2f]" % (normalizeAnglePIPI(bestyawobj['yaw'])))
                        self.turn(bestyawobj['yaw'] - self.yaw, timeout=timedelta(seconds=40))
                        self.go_straight(4)
                    except MoonException as e:
                        print(self.sim_time, self.robot_name, "Exception while performing initial going away from obstacles: ", str(e))


            print(self.sim_time, self.robot_name, "Asking hauler to arrive to <follow> distance")

            self.send_request('external_command hauler_1 follow') # force hauler to come real close, this will make it lose visual match with processing plant
            self.wait_for_hauler()


            # first, turn away from hauler
            # turn 360deg until seen, mark angle where the bbox was the biggest, then turn 180deg from there (want to face away as much as possible)
            # then, when hauler turns towards us (and follows?), it should have the same yaw which we can then share with it


            def angle_grouper(iterable):
                prev = None
                group = []
                for item in iterable:
                    if not prev or abs(normalizeAnglePIPI(item['yaw'] - prev['yaw'])) <= GROUP_ANGLES_MAX_DIFF:
                        group.append(item)
                    else:
                        yield group
                        group = [item]
                    prev = item
                if group:
                    yield group

            print(self.sim_time, self.robot_name, "Scanning 360 degrees of horizon to find hauler (widest wedge with continuous hauler detection)")

            self.visual_navigation_enabled = True
            while True:
                while True:
                    try:
                        self.turn(math.radians(30), ang_speed=0.8*self.max_angular_speed, with_stop=False, timeout=timedelta(seconds=40)) # get the turning stared
                        self.full_360_objects = {}
                        self.turn(math.radians(360), ang_speed=0.8*self.max_angular_speed, with_stop=False, timeout=timedelta(seconds=40))
                        if "rover" not in self.full_360_objects.keys() or len(self.full_360_objects["rover"]) == 0:
                            print(self.sim_time, self.robot_name, "Hauler not found during 360 turn")
                        else:
                            clusters = list(enumerate(angle_grouper(self.full_360_objects["rover"])))
                            if self.debug:
                                print(self.sim_time, self.robot_name, "Cluster list: ", str(clusters))
                            biggest_wedge_obj = max(clusters, key=lambda x: len(x[1]))
                            sum_sin = sum([math.sin(a['yaw']) for a in biggest_wedge_obj[1]])
                            sum_cos = sum([math.cos(a['yaw']) for a in biggest_wedge_obj[1]])
                            mid_angle = math.atan2(sum_sin, sum_cos)
                            print(self.sim_time, self.robot_name, "excavator: away-from-hauler angle (internal coordinates) [%.2f]" % (normalizeAnglePIPI(math.pi + mid_angle)))
                            turn_needed = normalizeAnglePIPI(math.pi + mid_angle - self.yaw)
                            # turn a little less to allow for extra turn after the effort was stopped
                            self.turn(turn_needed if turn_needed >= 0 else (turn_needed + 2*math.pi), ang_speed=0.8*self.max_angular_speed, timeout=timedelta(seconds=40))
                        break
                    except MoonException as e:
                        print(self.sim_time, self.robot_name, "Exception while performing initial turn: ", str(e))

                # TODO: this will be removed, shouldn't be needed
                if "rover" not in self.full_360_objects.keys() or len(self.full_360_objects["rover"]) == 0:
                    # hauler was not found visually but must be close, perform random walk
                    min_dist_obj = min(self.full_360_distances, key=lambda x: x['distance'])
                    self.turn(min_dist_obj['yaw'] - self.yaw, timeout=timedelta(seconds=40))
                    self.go_straight(3)
                    while abs(self.pitch) > 0.2:
                        self.publish("desired_movement", [GO_STRAIGHT, 0, self.default_effort_level])
                        self.wait(timedelta(milliseconds=100))
                    self.publish("desired_movement", [0, 0, 0])
                else:
                    break

            self.visual_navigation_enabled = False

            self.auto_light_adjustment = True
            # self.set_light_intensity("0.4")

            print(self.sim_time, self.robot_name, "Resetting VSLAM, requesting true_pose")

            # this will also trigger true pose once slam starts sending data
            self.send_request('vslam_reset', vslam_reset_time)

            while self.vol_list is None or not self.true_pose:
                try:
                    self.wait(timedelta(seconds=1))
                except MoonException as e:
                    print(self.sim_time, self.robot_name, "Exception while waiting for true pose: ", str(e))

            # calculate position behind the excavator to send hauler to, hauler will continue visually from there
            v = np.asmatrix(np.asarray([self.xyz[0], self.xyz[1], 1]))
            c, s = np.cos(self.yaw), np.sin(self.yaw)
            R = np.asmatrix(np.array(((c, -s, 0), (s, c, 0), (0, 0, 1))))
            T = np.asmatrix(np.array(((1, 0, -8),(0, 1, 0),(0, 0, 1))))
            pos =  np.dot(R, np.dot(T, np.dot(R.I, v.T)))
            #self.send_request('external_command hauler_1 goto %.1f %.1f %.2f' % (pos[0], pos[1], self.yaw))

            #self.send_request('external_command hauler_1 align %f' % self.yaw) # TODO: due to skidding, this yaw may still change

            self.send_request('external_command hauler_1 set_yaw %f' % self.yaw) # tell hauler looking at rover

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
                except MoonException as e:
                    print(self.sim_time, self.robot_name, "Exception while waiting for hauler to arrive: ", str(e))

            self.hauler_ready = False
            self.send_request('external_command hauler_1 approach %f' % self.yaw)

            # do not brake here, it will allow robots to align better
            while not self.hauler_ready:
                try:
                    self.wait(timedelta(seconds=3))
                except MoonException as e:
                    print(self.sim_time, self.robot_name, "Exception while waiting for hauler to arrive: ", str(e))

            self.hauler_ready = False

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


            def pursue_volatile():
                v = self.current_volatile
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
                                except MoonException as e:
                                    print(self.sim_time, self.robot_name, "Exception while waiting for VSLAM reset: ", str(e))

                            while not self.hauler_ready:
                                try:
                                    self.wait(timedelta(seconds=3))
                                except MoonException as e:
                                    print(self.sim_time, self.robot_name, "Exception while waiting for hauler to be ready: ", str(e))

                            self.hauler_ready = False

                            self.send_request('external_command hauler_1 request_true_pose')
                            try:
                                self.wait(timedelta(seconds=5)) # wait for pose to update (TODO: wait for a flag instead)
                            except MoonException as e:
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
                            self.going_to_volatile = True
                            try:
                                self.go_to_location(v, self.default_effort_level, offset=ARRIVAL_OFFSET, full_turn=True, timeout=timedelta(minutes=5), tolerance=ARRIVAL_TOLERANCE)
                            except NewVolatileTargetException:
                                raise
                            finally:
                                self.going_to_volatile = False
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
                start_alignment = self.sim_time
                while self.sim_time - start_alignment < timedelta(seconds=20) and self.mount_angle is None or abs(normalizeAnglePIPI(math.pi - self.mount_angle)) > 0.2:
                    try:
                        self.wait(timedelta(seconds=1))
                    except MoonException as e:
                        print(self.sim_time, self.robot_name, "Exception while waiting for arm to align: ", str(e))

                self.send_request('external_command hauler_1 onhold')
                try:
                    self.wait(timedelta(seconds=8))
                except MoonException as e:
                    print(self.sim_time, self.robot_name, "Exception while waiting for hauler to assume position: ", str(e))


                start_pose = self.xyz
                start_time = self.sim_time
                effort = self.default_effort_level/2 if distal_angle < DISTAL_THRESHOLD else -self.default_effort_level/2
                if abs(normalizeAnglePIPI(mount_angle)) < math.pi/2:
                    self.publish("desired_movement", [GO_STRAIGHT, int(100*math.degrees(normalizeAnglePIPI(mount_angle))), effort])
                else:
                    self.publish("desired_movement", [
                        GO_STRAIGHT,
                        int(100*math.degrees(normalizeAnglePIPI(math.copysign(math.pi - abs(normalizeAnglePIPI(mount_angle)), -normalizeAnglePIPI(mount_angle))))),
                        -effort])

                while distance(start_pose, self.xyz) < 0.7: # go 0.7m closer to volatile
                    try:
                        self.wait(timedelta(milliseconds=100))
                    except MoonException as e:
                        print(self.sim_time, self.robot_name, "Exception while adjusting position: ", str(e))
                        traceback.print_exc(file=sys.stdout)


                self.publish("desired_movement", [0,0,0])
                try:
                    self.wait(timedelta(seconds=3))
                except MoonException as e:
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

            while len(self.vol_list) > 0:
                self.current_volatile = self.get_next_volatile()
                if self.current_volatile is None:
                    break
                while True:
                    try:
                        pursue_volatile()
                    except NewVolatileTargetException:
                        continue
                    break
                v = self.current_volatile
                self.current_volatile = None

                accum = 0 # to check if we picked up 100.0 total and can finish
                for attempt_offset in [{'dist': 0.0, 'start_dig_angle': 0.0}, {'dist': 4, 'start_dig_angle': math.pi}, {'dist': -8, 'start_dig_angle': 0.0}]:
                    starting_point = self.xyz
                    starting_time = self.sim_time
                    while abs(abs(attempt_offset['dist']) - distance(self.xyz, starting_point)) > 1 and self.sim_time - starting_time < timedelta(seconds=15):
                        try:
                            self.go_straight(attempt_offset['dist'] - math.copysign(distance(self.xyz, starting_point), attempt_offset['dist']))
                        except MoonException as e:
                            # excess pitch or other exception could happen but we are stopped so should stabilize
                            print(self.sim_time, self.robot_name, "Exception when adjusting linearly", str(e))

                    self.send_speed_cmd(0.0, 0.0)

                    print ("---- VOLATILE REACHED ----")

                    try:
                        self.wait(timedelta(seconds=2))
                    except MoonException as e:
                        print(self.sim_time, self.robot_name, "Exception while waiting to come to stop: ", str(e))

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

                    found_angle = None

                    # mount 0.4-1.25 or 1.85-2.75 means we can't reach under the vehicle because of the wheels
                    mount_angle_sequence = [0.0, 0.39, -0.39, -0.83, 0.83, 1.26, -1.26, -1.55, 1.55, 1.84, -1.84, 2.31, -2.31, -2.76, 2.76, 3.14]
                    for a in mount_angle_sequence:
                        self.publish("bucket_dig", [a + attempt_offset['start_dig_angle'], 'append'])

                    while found_angle is None:
                        best_mount_angle = None
                        best_angle_volume = 0
                        best_distal_angle = None

                        self.set_brakes(True)
                        exit_time = self.sim_time + timedelta(seconds=250)
                        while found_angle is None and self.sim_time < exit_time:
                            # TODO instead of/in addition to timeout, trigger exit by bucket reaching reset position
                            try:
                                self.wait(timedelta(milliseconds=100))
                            except MoonException as e:
                                print(self.sim_time, self.robot_name, "Exception while waiting in loop: ", str(e))

                            if self.volatile_dug_up[1] != 100:
                                # we found volatile
                                if best_mount_angle is None:
                                    nma = normalizeAnglePIPI(self.mount_angle)
                                    if -math.pi/2 <= nma <= math.pi/2:
                                        # if mount angle is in the front half of the vehicle, approach directly from the back
                                        approach_angle = 0.0
                                    elif nma < -math.pi/2:
                                        approach_angle = normalizeAnglePIPI(nma + math.pi/2) # rover clockwise 90 degrees
                                    elif nma > math.pi/2:
                                        approach_angle = normalizeAnglePIPI(nma - math.pi/2) # rover counterclockwise 90 degrees

                                    # point arm in the hauler approach direction so that hauler aligns with the center of the excavator
                                    self.publish("bucket_dig", [normalizeAnglePIPI(approach_angle + math.pi), 'standby']) # lift arm and clear rest of queue
                                    self.wait_for_arm(normalizeAnglePIPI(approach_angle + math.pi))

                                    self.hauler_ready = False
                                    self.send_request('external_command hauler_1 approach %f' % normalizeAnglePIPI(self.yaw + approach_angle))

                                    # NOTE: this approach may bump excavator, scoop volume may worsen
                                    while not self.hauler_ready:
                                        try:
                                            self.wait(timedelta(seconds=1))
                                        except MoonException as e:
                                            print(self.sim_time, self.robot_name, "Exception while waiting for hauler to arrive: ", str(e))
                                    self.hauler_ready = False


                                    # wait for hauler to arrive, get its yaw, then we know which direction to drop
                                    drop_angle = normalizeAnglePIPI((self.hauler_yaw - self.yaw) - math.pi)
                                    print(self.sim_time, self.robot_name, "Drop angle set to: %.2f" % drop_angle)

                                    exit_time = self.sim_time + timedelta(seconds=80)
                                    self.publish("bucket_drop", [drop_angle, 'append']) # this will be duplicated below but need to queue now or load would be lost by another dig happening first
                                    self.publish("bucket_dig", [nma - 0.3, 'append'])
                                    self.publish("bucket_dig", [nma + 0.3, 'append'])
                                    # TODO: this plan doesn't end once these moves are executed, only with timeout
                                    best_distal_angle = self.first_distal_angle
                                    best_angle_volume = self.volatile_dug_up[2]
                                    best_mount_angle = nma

                                elif self.volatile_dug_up[2] > best_angle_volume:
                                    best_distal_angle = self.first_distal_angle
                                    best_angle_volume = self.volatile_dug_up[2]
                                    best_mount_angle = self.mount_angle

                                accum += self.volatile_dug_up[2]
                                if self.volatile_dug_up[2] > DIG_GOOD_LOCATION_MASS:
                                    found_angle = best_mount_angle
                                # go for volatile drop (to hauler), wait until finished
                                self.publish("bucket_drop", [drop_angle, 'prepend'])
                                while self.volatile_dug_up[1] != 100:
                                    try:
                                        self.wait(timedelta(milliseconds=300))
                                    except MoonException as e:
                                        print(self.sim_time, self.robot_name, "Exception while waiting to empty bin: ", str(e))
                                self.first_distal_angle = None

                        self.set_brakes(False)

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
                                except MoonException as e:
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
                                except MoonException as e:
                                    print(self.sim_time, self.robot_name, "Exception while waiting to empty bin: ", str(e))

                            if abs(accum - 100.0) < 0.0001:
                                print(self.sim_time, self.robot_name, "Volatile amount %.2f transfered, terminating" % accum)
                                # TODO: adjust transformation matrix using real location of the volatile
                                # TODO: assumes 100 to be total volatile at location, actual total reported initially, parse and match instead
                                return True

                    full_success = False
                    if found_angle is not None:
                        # have best position, dig everything up, adjust position, move to next volatile
                        self.set_brakes(True)
                        if scoop_all(found_angle):
                            self.vol_list.remove(v)# once scooping finished or given up on, remove volatile from list

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
                        self.set_brakes(False)


                    # wait for bucket to be dropped and/or position reset before moving on
                    self.publish("bucket_drop", [math.pi, 'reset'])
                    while self.volatile_dug_up[1] != 100 or (self.mount_angle is None or abs(normalizeAnglePIPI(math.pi - self.mount_angle)) > 0.2):
                        try:
                            self.wait(timedelta(seconds=1))
                        except MoonException as e:
                            print(self.sim_time, self.robot_name, "Exception while waiting to empty bin: ", str(e))

                    if full_success:
                        break # do not de-attempt 2m back and 2m front

                    # otherwise re-follow after giving up on this position
                    self.send_request('external_command hauler_1 follow')
                # end of for(0,+2,-4)

                if accum == 0 and not self.hauler_pose_requested:
                    self.set_brakes(True)

                    self.send_request('external_command hauler_1 approach %f' % self.hauler_yaw)
                    while not self.hauler_ready:
                        try:
                            self.wait(timedelta(seconds=1))
                        except MoonException as e:
                            print(self.sim_time, self.robot_name, "Exception while waiting for hauler to arrive: ", str(e))

                    self.hauler_ready = False

                    self.send_request('external_command hauler_1 request_true_pose')
                    try:
                        self.wait(timedelta(seconds=5)) # wait for pose to update (TODO: wait for a flag instead)
                    except MoonException as e:
                        print(self.sim_time, self.robot_name, "Exception while waiting to true pose: ", str(e))

                    self.set_brakes(False)

                self.send_request('external_command hauler_1 follow') # re-follow after each volatile


            print("***************** VOLATILES EXHAUSTED ****************************")


        except BusShutdownException:
            pass

        print ("EXCAVATOR END")

# vim: expandtab sw=4 ts=4
