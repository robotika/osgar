#!/usr/bin/env python2

from __future__ import print_function

import math
import socket
import sys
import threading
import time
import operator
import functools

import rospy
import rostopic
import tf
import zmq
import numpy as np

from sensor_msgs.msg import Imu, LaserScan, PointCloud2
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, Bool, Int32, String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import BatteryState, FluidPressure
from octomap_msgs.msg import Octomap
from subt_msgs.srv import PoseFromArtifact
from rtabmap_ros.msg import RGBDImage

sys.path.append("/osgar-ws/src/osgar/osgar/lib")
import serialize as osgar_serialize
from quaternion import multiply as multiply_quaternions, rotate_vector

def py3round(f):
    if abs(round(f) - f) == 0.5:
        return int(2.0 * round(f / 2.0))
    return int(round(f))


class Bus:
    def __init__(self):
        self.lock = threading.Lock()
        context = zmq.Context.instance()
        self.push = context.socket(zmq.PUSH)
        self.push.setsockopt(zmq.LINGER, 100)  # milliseconds
        self.push.bind('tcp://*:5565')
        self.pull = context.socket(zmq.PULL)
        self.pull.LINGER = 100
        self.pull.RCVTIMEO = 100
        self.pull.bind('tcp://*:5566')

    def register(self, *outputs):
        pass

    def publish(self, channel, data):
        raw = osgar_serialize.serialize(data)
        with self.lock:
            self.push.send_multipart([channel, raw])

    def listen(self):
        while not rospy.is_shutdown():
            try:
                channel, bytes_data = self.pull.recv_multipart()
                data = osgar_serialize.deserialize(bytes_data)
                return channel.decode('ascii'), data
            except zmq.ZMQError as e:
                if e.errno != zmq.EAGAIN:
                    rospy.logerr("zmq error")
                    sys.exit("zmq error")
        rospy.loginfo("done")
        sys.exit()


def empty(data):
    return Empty()


def twist(data):
    linear, angular = data
    vel_msg = Twist()
    vel_msg.linear.x, vel_msg.linear.y, vel_msg.linear.z = linear
    vel_msg.angular.x, vel_msg.angular.y, vel_msg.angular.z = angular
    return vel_msg


class main:
    def __init__(self, robot_name, robot_config, robot_is_marsupial):
        rospy.init_node('cloudsim2osgar', log_level=rospy.DEBUG)
        self.bus = Bus()
        self.robot_name = robot_name
        self.prev_gas_detected = None  # report on change including the first reading

        # common topics
        topics = [
            ('/' + robot_name + '/battery_state', BatteryState, self.battery_state, ('battery_state',)),
            ('/subt/score', Int32, self.score, ('score',)),
            ('/' + robot_name + '/gas_detected', Bool, self.gas_detected, ('gas_detected', )),
        ]

        publishers = {}
        publishers['cmd_vel'] = (rospy.Publisher('/' + robot_name + '/cmd_vel', Twist, queue_size=1), twist)

        self.tf = tf.TransformListener()

        # configuration specific topics
        robot_description = rospy.get_param("/{}/robot_description".format(robot_name))
        if robot_config == "ROBOTIKA_X2_SENSOR_CONFIG_1":
            rospy.loginfo("robotika x2")
        elif robot_config == "SSCI_X4_SENSOR_CONFIG_2":
            rospy.loginfo("ssci drone")
            topics.append(('/' + robot_name + '/top_scan', LaserScan, self.top_scan, ('top_scan',)))
            topics.append(('/' + robot_name + '/bottom_scan', LaserScan, self.bottom_scan, ('bottom_scan',)))
            topics.append(('/' + robot_name + '/odom_fused', Odometry, self.odom_fused, ('pose3d',)))
            topics.append(('/' + robot_name + '/air_pressure', FluidPressure, self.air_pressure, ('air_pressure',)))
            topics.append(('/' + robot_name + '/front_scan', LaserScan, self.scan360, ('scan360',)))
            topics.append(('/rtabmap/rgbd/compressed', RGBDImage, self.rgbd_front, ('rgbd_front',)))
            if robot_name.endswith('XM'):
                topics.append(('/mapping/octomap_binary', Octomap, self.octomap, ('octomap',)))
            if robot_is_marsupial == 'true':
                rospy.loginfo("X4 is marsupial")
                publishers['detach'] = (rospy.Publisher('/' + robot_name + '/detach', Empty, queue_size=1), empty)
        elif robot_config == "TEAMBASE":
            rospy.loginfo("teambase")
        elif robot_config.startswith("ROBOTIKA_FREYJA_SENSOR_CONFIG"):
            if robot_config.endswith("_2"):
                rospy.loginfo("freya 2 (with comms beacons)")
                publishers['deploy'] = (rospy.Publisher('/' + robot_name + '/breadcrumb/deploy', Empty, queue_size=1), empty)
            else:
                rospy.loginfo("freya 1 (basic)")
            topics.append(('/' + robot_name + '/odom_fused', Odometry, self.odom_fused, ('pose3d',)))
            topics.append(('/' + robot_name + '/scan_front', LaserScan, self.scan_front, ('scan_front',)))
            topics.append(('/' + robot_name + '/scan_rear', LaserScan, self.scan_rear, ('scan_rear',)))
            topics.append(('/' + robot_name + '/local_map/output/scan', LaserScan, self.scan360, ('scan360',)))
            topics.append(('/rtabmap/rgbd/front/compressed', RGBDImage, self.rgbd_front, ('rgbd_front',)))
            topics.append(('/rtabmap/rgbd/rear/compressed', RGBDImage, self.rgbd_rear, ('rgbd_rear',)))
            if robot_name.endswith('XM'):
                topics.append(('/mapping/octomap_binary', Octomap, self.octomap, ('octomap',)))
        elif robot_config.startswith("ROBOTIKA_KLOUBAK_SENSOR_CONFIG"):
            if robot_config.endswith("_2"):
                rospy.loginfo("k2 2 (with comms beacons)")
                publishers['deploy'] = (rospy.Publisher('/' + robot_name + '/breadcrumb/deploy', Empty, queue_size=1), empty)
            else:
                rospy.loginfo("k2 1 (basic)")
            topics.append(('/' + robot_name + '/odom_fused', Odometry, self.odom_fused, ('pose3d',)))
            topics.append(('/' + robot_name + '/scan_front', LaserScan, self.scan_front, ('scan_front',)))
            topics.append(('/' + robot_name + '/scan_rear', LaserScan, self.scan_rear, ('scan_rear',)))
            topics.append(('/rtabmap/rgbd/front/compressed', RGBDImage, self.rgbd_front, ('rgbd_front',)))
            topics.append(('/rtabmap/rgbd/rear/compressed', RGBDImage, self.rgbd_rear, ('rgbd_rear',)))
        else:
            rospy.logerr("unknown configuration")
            return

        if robot_config.startswith("ROBOTIKA_KLOUBAK_SENSOR_CONFIG"):
            topics.append(('/' + robot_name + '/imu/front/data', Imu, self.imu, ('acc',)))
        else:
            topics.append(('/' + robot_name + '/imu/data', Imu, self.imu, ('acc',)))

        outputs = functools.reduce(operator.add, (t[-1] for t in topics)) + ('robot_name',)
        self.bus.register(outputs)

        # origin
        self.origin = None
        origin_retry_delay = 0.2
        ORIGIN_RETRY_EXPONENTIAL_BACKOFF = 1.3
        MAX_ORIGIN_RETRY_DELAY = 2.0
        ORIGIN_SERVICE_NAME = "/subt/pose_from_artifact_origin"
        rospy.wait_for_service(ORIGIN_SERVICE_NAME)
        origin_service = rospy.ServiceProxy(ORIGIN_SERVICE_NAME, PoseFromArtifact)
        origin_request = String()
        origin_request.data = robot_name
        while self.origin is None:
            try:
                origin_response = origin_service(origin_request)
                if origin_response.success:
                    origin_pose = origin_response.pose.pose
                    origin_position = origin_response.pose.pose.position
                    origin_orientation = origin_response.pose.pose.orientation
                    self.origin = (
                            (origin_position.x, origin_position.y, origin_position.z),
                            (origin_orientation.x, origin_orientation.y, origin_orientation.z,
                                origin_orientation.w))
            except rospy.ServiceException:
                rospy.logerr("Failed to get origin. Trying again.")
                time.sleep(origin_retry_delay)
                origin_retry_delay = min(
                        MAX_ORIGIN_RETRY_DELAY,
                        ORIGIN_RETRY_EXPONENTIAL_BACKOFF * origin_retry_delay)

        for name, type, handler, _ in topics:
            rospy.loginfo("waiting for {}".format(name))
            rospy.wait_for_message(name, type)
            setattr(self, handler.__name__+"_count", 0)
            rospy.Subscriber(name, type, handler)

        self.bus.publish('robot_name', robot_name)

        # main thread receives data from osgar and sends it to ROS
        while True:
            channel, data = self.bus.listen()
            # switch on channel to feed different ROS publishers
            if channel in publishers:
                publisher, handle = publishers[channel]
                publisher.publish(handle(data))

                if channel != 'cmd_vel':
                    # for debugging breadcrumbs deployment and marsupial detachment
                    rospy.logdebug("receiving: {} {}".format(channel, data))
            else:
                rospy.loginfo("ignoring: {} {}".format(channel, data))

    def imu(self, msg):
        self.imu_count += 1
        rospy.loginfo_throttle(10, "imu callback: {}".format(self.imu_count))
        acc = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]

        self.bus.publish('acc', [py3round(x * 1000) for x in acc])
        # preliminary suggestion for combined message
        #angular_velocity = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
        #data = [
        #    msg.header.stamp.to_nsec()/1000000, # time in milliseconds
        #    orientation,
        #    angular_velocity,
        #    acc,
        #]
        #self.bus.publish('imu', data)

    def top_scan(self, msg):
        self.top_scan_count += 1
        rospy.loginfo_throttle(10, "top_scan callback: {}".format(self.top_scan_count))
        self.bus.publish('top_scan', msg.ranges)

    def bottom_scan(self, msg):
        self.bottom_scan_count += 1
        rospy.loginfo_throttle(10, "bottom_scan callback: {}".format(self.bottom_scan_count))
        self.bus.publish('bottom_scan', msg.ranges)

    def odom_fused(self, msg):
        self.odom_fused_count += 1
        rospy.loginfo_throttle(10, "odom_fused callback: {}".format(self.odom_fused_count))

        raw_position = msg.pose.pose.position
        raw_orientation = msg.pose.pose.orientation

        raw_xyz = [raw_position.x, raw_position.y, raw_position.z]
        raw_quat = [raw_orientation.x, raw_orientation.y, raw_orientation.z, raw_orientation.w]

        origin_translation, origin_rotation = self.origin

        full_orientation = multiply_quaternions(origin_rotation, raw_quat)
        full_translation = [sum(v) for v in zip(
            origin_translation,
            rotate_vector(raw_xyz, origin_rotation))]

        self.bus.publish('pose3d', [full_translation, full_orientation])

    def battery_state(self, msg):
        self.battery_state_count += 1
        rospy.loginfo_throttle(10, "battery_state callback: {}".format(self.battery_state_count))
        if getattr(self, 'last_battery_secs', None) != msg.header.stamp.secs:
            self.bus.publish("battery_state", msg.percentage)
            self.last_battery_secs = msg.header.stamp.secs

    def score(self, msg):
        self.score_count += 1
        rospy.loginfo_throttle(10, "score callback: {}".format(self.score_count))
        self.bus.publish("score", msg.data)

    def air_pressure(self, msg):
        self.air_pressure_count += 1
        rospy.loginfo_throttle(10, "air_pressure callback: {}".format(self.air_pressure_count))
        self.bus.publish('air_pressure', msg.fluid_pressure)

    def scan_front(self, msg):
        self.scan_front_count += 1
        rospy.loginfo_throttle(10, "scan_front callback: {}".format(self.scan_front_count))
        scan = [int(x * 1000) if msg.range_min < x < msg.range_max else 0 for x in msg.ranges]
        self.bus.publish('scan_front', scan)

    def scan_rear(self, msg):
        self.scan_rear_count += 1
        rospy.loginfo_throttle(10, "scan_rear callback: {}".format(self.scan_rear_count))
        scan = [int(x * 1000) if msg.range_min < x < msg.range_max else 0 for x in msg.ranges]
        self.bus.publish('scan_rear', scan)

    def scan360(self, msg):
        # 3rd copy, i.e. almost time for refactoring ...
        self.scan360_count += 1
        rospy.loginfo_throttle(10, "scan360 callback: {}".format(self.scan360_count))
        scan = [int(x * 1000) if msg.range_min < x < msg.range_max else 0 for x in msg.ranges]
        self.bus.publish('scan360', scan)

    def convert_depth(self, msg):
        assert msg.encoding == '32FC1', msg.encoding  # unsupported encoding
        # depth is array of floats, OSGAR uses uint16 in millimeters
        # cut min & max (-inf and inf are used for clipping)
        arr = np.frombuffer(msg.data, dtype=np.dtype('f')) * 1000
        arr = np.clip(arr, 1, 0xFFFF)
        arr = np.ndarray.astype(arr, dtype=np.dtype('H'))
        return np.array(arr).reshape((msg.height, msg.width))

    def convert_rgbd(self, msg):
        # Pose of the robot relative to starting position.
        try:
            robot_pose = self.tf.lookupTransform('odom', self.robot_name, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return None
        # Pose of the robot in the world coordinate frame, i.e. likely a non-zero initial
        # position.
        if self.origin is not None:
            origin_translation, origin_rotation = self.origin
            raw_robot_translation, raw_robot_rotation = robot_pose
            full_robot_rotation = multiply_quaternions(origin_rotation, raw_robot_rotation)
            full_robot_translation = [sum(v) for v in zip(
                origin_translation,
                rotate_vector(raw_robot_translation, origin_rotation))]
            robot_pose = full_robot_translation, full_robot_rotation
        # Position of the camera relative to the robot.
        try:
            # RTABMAP produces rotation in a visual coordinate frame (Z axis
            # forward), but we need to match the world frame (X axis forward)
            # of robot_pose.
            camera_frame_id = msg.header.frame_id.replace("_optical", "")
            camera_pose = self.tf.lookupTransform(
                    self.robot_name, camera_frame_id, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return None
        rgb = msg.rgb_compressed.data
        d = msg.depth_compressed.data
        return (robot_pose, camera_pose, rgb, d)

    def rgbd_front(self, msg):
        self.rgbd_front_count += 1
        rospy.loginfo_throttle(10, "rgbd_front callback: {}".format(self.rgbd_front_count))
        rgbd = self.convert_rgbd(msg)
        if rgbd is not None:
            self.bus.publish('rgbd_front', rgbd)

    def rgbd_rear(self, msg):
        self.rgbd_rear_count += 1
        rospy.loginfo_throttle(10, "rgbd_rear callback: {}".format(self.rgbd_rear_count))
        rgbd = self.convert_rgbd(msg)
        if rgbd is not None:
            self.bus.publish('rgbd_rear', rgbd)

    def convert_points(self, msg):
        # accept only Velodyne VLC-16 (for the ver0)
        assert msg.height == 16, msg.height
        assert msg.width == 1800, msg.width
        assert msg.point_step == 32, msg.point_step
        assert msg.row_step == 57600, msg.row_step
        arr = np.frombuffer(msg.data, dtype=np.float32)
        points3d = arr.reshape((msg.height, msg.width, 8))[:, :, 0:3]  # keep only (x, y, z)
        return points3d

    def points(self, msg):
        self.points_count += 1
        rospy.loginfo_throttle(10, "points callback: {}".format(self.points_count))
        self.bus.publish('points', self.convert_points(msg))

    def octomap(self, msg):
        self.octomap_count += 1
        rospy.loginfo_throttle(10, "octomap callback: {}".format(self.octomap_count))
        self.bus.publish('octomap', msg.data)

    def gas_detected(self, msg):
        self.gas_detected_count += 1
        rospy.loginfo_throttle(10, "gas_detected callback: {}".format(self.gas_detected_count))
        detected = msg.data
        if detected != self.prev_gas_detected:
            self.bus.publish('gas_detected', detected)
            self.prev_gas_detected = detected


if __name__ == '__main__':
    if len(sys.argv) < 4:
        print("need robot name, config and is_marsupial as arguments", file=sys.stderr)
        sys.exit(2)
    try:
        main(sys.argv[1], sys.argv[2], sys.argv[3])
    except rospy.exceptions.ROSInterruptException:
        rospy.loginfo("shutdown")

