#!/usr/bin/env python2

from __future__ import print_function

import math
import socket
import sys
import threading
import operator
import functools

import rospy
import tf
import zmq
import numpy as np

from sensor_msgs.msg import Imu, LaserScan, PointCloud2
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, Bool, Int32
from geometry_msgs.msg import Pose, PoseArray, Twist
from sensor_msgs.msg import BatteryState, FluidPressure
from octomap_msgs.msg import Octomap
from rtabmap_ros.msg import RGBDImage
from visualization_msgs.msg import Marker, MarkerArray

sys.path.append("/osgar-ws/src/osgar/osgar/lib")
import serialize as osgar_serialize
from quaternion import euler_zyx

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


class MappingInformationProvider:
    ''' Provides data to DARPA's Mapping Server.

    https://github.com/osrf/subt/wiki/api#mapping-server
    '''
    def __init__(self):
        self.lock = threading.Lock()

        self.recent_trajectory = []
        self.trajectory_publisher = rospy.Publisher('/poses', PoseArray, queue_size=10)

        self.artifacts = []
        self.artf_publisher = rospy.Publisher('/markers', MarkerArray, queue_size=10)
        # Mapping server accepst at most one message every ten seconds.
        ARTIFACT_REPORT_INTERVAL = 0.11
        self.artf_timer = rospy.Timer(rospy.Duration(ARTIFACT_REPORT_INTERVAL), self.publish_artifacts)

    def add_pose_to_trajectory(self, pose, timestamp):
        self.recent_trajectory.append((timestamp, pose))
        # Mapping server accepts at most one message per second. We give
        # ourselves some margin to stay on the safe side.
        MIN_TRAJECTORY_SEGMENT_DURATION = rospy.Duration(1.2)
        if self.recent_trajectory[-1][0] - self.recent_trajectory[0][0] >= MIN_TRAJECTORY_SEGMENT_DURATION:
            segment = PoseArray()
            segment.header.frame_id = 'global'
            segment.header.stamp = self.recent_trajectory[-1][0]

            for _, (p, o) in self.recent_trajectory:
                info = Pose()
                position = info.position
                orientation = info.orientation
                position.x, position.y, position.z = p
                orientation.x, orientation.y, orientation.z, orientation.w = o
                segment.poses.append(info)

            self.trajectory_publisher.publish(segment)
            self.recent_trajectory = []

    def add_artifact(self, artf_type, artf_xyz):
        with self.lock:
            self.artifacts.append((artf_type, artf_xyz))

    def publish_artifacts(self, event):
        VISUALIZATION = {
                # TYPE: (label, rgb, shape, size)
                'TYPE_BACKPACK': ("Backpack", (1.0, 0.0, 0.0), Marker.SPHERE, (0.4, 0.4, 0.5)),
                'TYPE_CUBE':  ("Cube", (0.0, 1.0, 1.0), Marker.CUBE, (0.3, 0.3, 0.3)),
                'TYPE_DRILL': ("Drill", (1.0, 0.5, 1.0), Marker.CYLINDER, (0.2, 0.2, 0.3)),
                'TYPE_EXTINGUISHER': ("Extinguisher", (1.0, 0.0, 0.0), Marker.CYLINDER, (0.3, 0.3, 0.5)),
                'TYPE_GAS': ("Gas", (1.0, 1.0, 1.0), Marker.SPHERE, (1.0, 1.0, 1.0)),
                'TYPE_HELMET': ("Helmet", (0.7, 0.7, 0.7), Marker.SPHERE, (0.3, 0.3, 0.3)),
                'TYPE_PHONE': ("Phone", (0.0, 1.0, 0.5), Marker.CUBE, (0.15, 0.15, 0.1)),
                'TYPE_RESCUE_RANDY': ("Survivor", (0.8, 1.0, 0.7), Marker.CYLINDER, (0.8, 0.8, 1.4)),
                'TYPE_ROPE': ("Rope", (0.0, 0.0, 1.0), Marker.CYLINDER, (0.5, 0.5, 0.2)),
                'TYPE_VENT': ("Vent", (1.0, 1.0, 1.0), Marker.CUBE, (0.8, 0.8, 0.8)),
                }

        msg = MarkerArray()
        now = rospy.Time.now()
        with self.lock:
            for artf_idx, (artf_type, artf_xyz) in enumerate(self.artifacts):
                artf_name, rgb, shape, size = VISUALIZATION[artf_type]

                marker = Marker()
                marker.header.frame_id = 'global'
                marker.header.stamp = now
                marker.ns = 'artifact:object'
                marker.id = artf_idx
                marker.action = Marker.MODIFY
                marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = [v / 1000.0 for v in artf_xyz]
                marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z, marker.pose.orientation.w = 0, 0, 0, 1
                marker.lifetime = rospy.Duration(0)  # FOREVER
                marker.frame_locked = False
                marker.type = shape
                marker.scale.x, marker.scale.y, marker.scale.z = size
                marker.color.r, marker.color.g, marker.color.b = rgb
                marker.color.a = 0.3

                msg.markers.append(marker)

                label = Marker()
                label.header.frame_id = 'global'
                label.header.stamp = now
                label.ns = 'artifact:name'
                label.id = artf_idx
                label.action = Marker.MODIFY
                label.text = artf_name
                label.pose.position.x, label.pose.position.y, label.pose.position.z = [v / 1000.0 for v in artf_xyz]
                label.pose.position.z += 1.2  # Showing the label above the object.
                label.pose.orientation.x, label.pose.orientation.y, label.pose.orientation.z, label.pose.orientation.w = 0, 0, 0, 1
                label.lifetime = rospy.Duration(0)  # FOREVER
                label.frame_locked = False
                label.type = Marker.TEXT_VIEW_FACING
                label.scale.x, label.scale.y, label.scale.z = 1, 1, 1
                label.color.r, label.color.g, label.color.b = rgb
                label.color.a = 1.0

                msg.markers.append(label)


        self.artf_publisher.publish(msg)


class main:
    def __init__(self, robot_name, robot_config, robot_is_marsupial):
        rospy.init_node('cloudsim2osgar', log_level=rospy.DEBUG)
        self.bus = Bus()
        self.robot_name = robot_name
        self.robot_config = robot_config
        self.prev_gas_detected = None  # report on change including the first reading

        self.mapping_info_provider = MappingInformationProvider()

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
        elif robot_config == "CORO_PAM_SENSOR_CONFIG_1":
            rospy.loginfo("CoRo Pam drone")
            topics.append(('/' + robot_name + '/local_map/output/up', LaserScan, self.top_scan, ('top_scan',)))
            topics.append(('/' + robot_name + '/local_map/output/down', LaserScan, self.bottom_scan, ('bottom_scan',)))
            topics.append(('/' + robot_name + '/odom_fused', Odometry, self.odom_fused, ('pose3d',)))
            topics.append(('/' + robot_name + '/air_pressure', FluidPressure, self.air_pressure, ('air_pressure',)))
            topics.append(('/rtabmap/rgbd/front/compressed', RGBDImage, self.rgbd_front, ('rgbd_front',)))
            topics.append(('/rtabmap/rgbd/left/compressed_rotated', RGBDImage, self.rgbd_left, ('rgbd_left',)))
            topics.append(('/rtabmap/rgbd/right/compressed_rotated', RGBDImage, self.rgbd_right, ('rgbd_right',)))
            topics.append(('/' + robot_name + '/local_map/output/scan', LaserScan, self.scan360, ('scan360',)))
            if robot_name.endswith('XM'):
                topics.append(('/mapping/octomap_binary', Octomap, self.octomap, ('octomap',)))
            if robot_is_marsupial == 'true':
                rospy.loginfo("CoRo Pam is marsupial")
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
            topics.append(('/' + robot_name + '/local_map/output/scan', LaserScan, self.scan360, ('scan360',)))
            topics.append(('/rtabmap/rgbd/front/compressed', RGBDImage, self.rgbd_front, ('rgbd_front',)))
            topics.append(('/rtabmap/rgbd/rear/compressed', RGBDImage, self.rgbd_rear, ('rgbd_rear',)))
        else:
            rospy.logerr("unknown configuration")
            return

        if robot_config.startswith("ROBOTIKA_KLOUBAK_SENSOR_CONFIG"):
            topics.append(('/' + robot_name + '/imu/front/data', Imu, self.imu, ('acc',)))
        else:
            topics.append(('/' + robot_name + '/imu/data', Imu, self.imu, ('acc',)))

        outputs = functools.reduce(operator.add, (t[-1] for t in topics)) + ('robot_name', 'joint_angle')
        self.bus.register(outputs)

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
            elif channel == 'artf_xyz':
                for artf_type, artf_xyz, _, _ in data:
                    self.mapping_info_provider.add_artifact(artf_type, artf_xyz)
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

        # We want pose of the robot in the world coordinate system with (0, 0, 0)
        # at the entrance gate. The incoming message is in the "odom" frame, i.e.
        # relative to the starting point of the robot. We need to transform the
        # coordinates one way or the other, so let's just directly request the
        # transform we need.
        try:
            self.tf.waitForTransform('global', self.robot_name, msg.header.stamp, rospy.Duration(0.05))
            robot_pose = self.tf.lookupTransform('global', self.robot_name, msg.header.stamp)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf.Exception) as e:
            rospy.logerr('tf global error: {}'.format(e))
            return None

        self.bus.publish('pose3d', robot_pose)

        if self.robot_config.startswith("ROBOTIKA_KLOUBAK_SENSOR_CONFIG"):
            try:
                body_rotation = euler_zyx(
                        self.tf.lookupTransform(self.robot_name + '/chassis_back', self.robot_name + '/chassis_front', rospy.Time(0))[1])[0]
                self.bus.publish('joint_angle', [int(100 * math.degrees(body_rotation))])  # Hundreth of a degree.
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.logerr('joint error: {}'.format(e))

        self.mapping_info_provider.add_pose_to_trajectory(robot_pose, msg.header.stamp)

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
        if msg.intensities:
            # We are misusing this field to store preferred flight slopes.
            slopes = [int(math.degrees(x) * 10) for x in msg.intensities]
            self.bus.publish('slopes', slopes)
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
            self.tf.waitForTransform('global', self.robot_name, msg.header.stamp, rospy.Duration(0.3))
            robot_pose = self.tf.lookupTransform('global', self.robot_name, msg.header.stamp)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf.Exception) as e:
            rospy.logerr('tf global error: {}'.format(e))
            return None
        # Position of the camera relative to the robot.
        try:
            # RTABMAP produces rotation in a visual coordinate frame (Z axis
            # forward), but we need to match the world frame (X axis forward)
            # of robot_pose.
            WORLD_TO_OPTICAL = tf.transformations.quaternion_from_matrix(
                    [[ 0, -1,  0, 0],
                     [ 0,  0, -1, 0],
                     [ 1,  0,  0, 0],
                     [ 0,  0,  0, 1]])
            self.tf.waitForTransform(self.robot_name, msg.header.frame_id, msg.header.stamp, rospy.Duration(0.3))
            camera_xyz, camera_quat = self.tf.lookupTransform(
                    self.robot_name, msg.header.frame_id, msg.header.stamp)
            camera_pose = (camera_xyz,
                    tf.transformations.quaternion_multiply(camera_quat, WORLD_TO_OPTICAL).tolist())
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf.Exception) as e:
            rospy.logerr('tf camera error: {}'.format(e))
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

    def rgbd_left(self, msg):
        self.rgbd_left_count += 1
        rospy.loginfo_throttle(10, "rgbd_left callback: {}".format(self.rgbd_left_count))
        rgbd = self.convert_rgbd(msg)
        if rgbd is not None:
            self.bus.publish('rgbd_left', rgbd)

    def rgbd_right(self, msg):
        self.rgbd_right_count += 1
        rospy.loginfo_throttle(10, "rgbd_right callback: {}".format(self.rgbd_right_count))
        rgbd = self.convert_rgbd(msg)
        if rgbd is not None:
            self.bus.publish('rgbd_right', rgbd)

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

