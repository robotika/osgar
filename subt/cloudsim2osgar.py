#!/usr/bin/env python2

from __future__ import print_function

import errno
import math
import socket
import sys
import threading
import time
import operator
import functools

import rospy
import rostopic
import zmq
import numpy as np

from sensor_msgs.msg import Imu, LaserScan, CompressedImage, Image, PointCloud2
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, Int32
from sensor_msgs.msg import BatteryState, FluidPressure

sys.path.append("/osgar-ws/src/osgar/osgar/lib")
import serialize as osgar_serialize

def py3round(f):
    if abs(round(f) - f) == 0.5:
        return int(2.0 * round(f / 2.0))
    return int(round(f))


def has_breadcrumbs(robot_name):
    # possibly fragile detection if robot has bredcrumbs
    rospy.sleep(1)
    _publishers, subscribers = rostopic.get_topic_list()
    for name, type, _ in subscribers:
        if name == "/{}/breadcrumb/deploy".format(robot_name):
            return True
    return False

def wait_for_master():
    """Return when /use_sim_time is set in rosparams. If rospy.init_node is called before /use_sim_time
    is set, it uses real time."""
    print("Waiting for rosmaster")
    start = time.time()
    last_params = []
    while not rospy.is_shutdown():
        try:
            params = rospy.get_param_names()
            if params != last_params:
                print(time.time()-start, params)
                if '/use_sim_time' in params:
                    return True
                last_params = params
            time.sleep(0.01)
        except socket.error as serr:
            if serr.errno != errno.ECONNREFUSED:
                raise serr  # re-raise error if its not the one we want
            time.sleep(0.5)

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


class main:
    def __init__(self, robot_name):
        # get cloudsim ready
        wait_for_master()
        rospy.init_node('cloudsim2osgar', log_level=rospy.DEBUG)
        self.bus = Bus()

        # common topics
        topics = [
            ('/' + robot_name + '/battery_state', BatteryState, self.battery_state, ('battery_state',)),
            ('/subt/score', Int32, self.score, ('score',)),
        ]

        publishers = {}

        # configuration specific topics
        robot_description = rospy.get_param("/{}/robot_description".format(robot_name))
        if "robotika_x2_sensor_config_1" in robot_description:
            rospy.loginfo("robotika x2")
        elif "ssci_x4_sensor_config_2" in robot_description:
            rospy.loginfo("ssci drone")
            topics.append(('/' + robot_name + '/top_scan', LaserScan, self.top_scan, ('top_scan',)))
            topics.append(('/' + robot_name + '/bottom_scan', LaserScan, self.bottom_scan, ('bottom_scan',)))
            topics.append(('/' + robot_name + '/odom_fused', Odometry, self.odom_fused, ('pose3d',)))
            topics.append(('/' + robot_name + '/air_pressure', FluidPressure, self.air_pressure, ('air_pressure',)))
        elif "TeamBase" in robot_description:
            rospy.loginfo("teambase")
        elif "robotika_freyja_sensor_config" in robot_description:
            if has_breadcrumbs(robot_name):
                rospy.loginfo("freya 2 (with comms beacons)")
                publishers['deploy'] = rospy.Publisher('/' + robot_name + '/breadcrumb/deploy', Empty, queue_size=1)
            else:
                rospy.loginfo("freya 1 (basic)")
            topics.append(('/' + robot_name + '/odom_fused', Odometry, self.odom_fused, ('pose3d',)))
            topics.append(('/' + robot_name + '/rgbd_front/image_raw/compressed', CompressedImage, self.image_front, ('image_front',)))
            topics.append(('/' + robot_name + '/rgbd_rear/image_raw/compressed', CompressedImage, self.image_rear, ('image_rear',)))
            topics.append(('/' + robot_name + '/scan_front', LaserScan, self.scan_front, ('scan_front',)))
            topics.append(('/' + robot_name + '/scan_rear', LaserScan, self.scan_rear, ('scan_rear',)))
            topics.append(('/' + robot_name + '/rgbd_front/depth', Image, self.depth_front, ('depth_front',)))
            topics.append(('/' + robot_name + '/rgbd_rear/depth', Image, self.depth_rear, ('depth_rear',)))
        elif "robotika_kloubak_sensor_config" in robot_description:
            if has_breadcrumbs(robot_name):
                rospy.loginfo("k2 2 (with comms beacons)")
                publishers['deploy'] = rospy.Publisher('/' + robot_name + '/breadcrumb/deploy', Empty, queue_size=1)
            else:
                rospy.loginfo("k2 1 (basic)")
            topics.append(('/' + robot_name + '/odom_fused', Odometry, self.odom_fused, ('pose3d',)))
        elif "explorer_r2_sensor_config" in robot_description:
            rospy.loginfo("explorer R2")
            topics.append(('/' + robot_name + '/rs_front/color/image/compressed', CompressedImage, self.image_front, ('image_front',)))
            topics.append(('/' + robot_name + '/rs_front/depth/image', Image, self.depth_front, ('depth_front',)))
            topics.append(('/' + robot_name + '/points', PointCloud2, self.points, ('points',)))
        else:
            rospy.logerr("unknown configuration")
            return

        if "robotika_kloubak_sensor_config" in robot_description:
            topics.append(('/' + robot_name + '/imu/front/data', Imu, self.imu, ('rot', 'acc', 'orientation')))
        else:
            topics.append(('/' + robot_name + '/imu/data', Imu, self.imu, ('rot', 'acc', 'orientation')))

        outputs = functools.reduce(operator.add, (t[-1] for t in topics))
        self.bus.register(outputs)

        for name, type, handler, _ in topics:
            rospy.loginfo("waiting for {}".format(name))
            rospy.wait_for_message(name, type)
            setattr(self, handler.__name__+"_count", 0)
            rospy.Subscriber(name, type, handler)

        # main thread receives data from osgar and sends it to ROS
        while True:
            channel, data = self.bus.listen()
            rospy.logdebug("receiving: {} {}".format(channel, data))
            # switch on channel to feed different ROS publishers
            if channel in publishers:
                publishers[channel].publish(*data)  # just guessing for Empty, where ([],) is wrong
            else:
                rospy.loginfo("ignoring: {} {}".format(channel, data))

    def imu(self, msg):
        self.imu_count += 1
        rospy.loginfo_throttle(10, "imu callback: {}".format(self.imu_count))
        acc = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
        orientation = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]

        # copy & paste from rosmsg
        q0, q1, q2, q3 = orientation  # quaternion
        x = math.atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2))
        y = math.asin(2 * (q0 * q2 - q3 * q1))
        z = math.atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3))
        rot = [x, y, z]
        # end of copy & paste from rosmsg

        self.bus.publish('rot', [py3round(math.degrees(angle) * 100) for angle in rot])
        self.bus.publish('acc', [py3round(x * 1000) for x in acc])
        self.bus.publish('orientation', orientation)
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

        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        xyz = [position.x, position.y, position.z]
        orientation = [orientation.x, orientation.y, orientation.z, orientation.w]

        self.bus.publish('pose3d', [xyz, orientation])

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

    def image_front(self, msg):
        self.image_front_count += 1
        rospy.loginfo_throttle(10, "image_front callback: {}".format(self.image_front_count))
        self.bus.publish('image_front', msg.data)

    def image_rear(self, msg):
        self.image_rear_count += 1
        rospy.loginfo_throttle(10, "image_rear callback: {}".format(self.image_rear_count))
        self.bus.publish('image_rear', msg.data)

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

    def convert_depth(self, msg):
        assert msg.encoding == '32FC1', msg.encoding  # unsupported encoding
        # depth is array of floats, OSGAR uses uint16 in millimeters
        # cut min & max (-inf and inf are used for clipping)
        arr = np.frombuffer(msg.data, dtype=np.dtype('f')) * 1000
        arr = np.clip(arr, 1, 0xFFFF)
        arr = np.ndarray.astype(arr, dtype=np.dtype('H'))
        return np.array(arr).reshape((msg.height, msg.width))

    def depth_front(self, msg):
        self.depth_front_count += 1
        rospy.loginfo_throttle(10, "depth_front callback: {}".format(self.depth_front_count))
        self.bus.publish('depth_front', self.convert_depth(msg))

    def depth_rear(self, msg):
        self.depth_rear_count += 1
        rospy.loginfo_throttle(10, "depth_rear callback: {}".format(self.depth_rear_count))
        self.bus.publish('depth_rear', self.convert_depth(msg))

    def convert_points(self, msg):
        # accept only Velodyne VLC-16 (for the ver0)
        assert msg.height == 16, msg.height
        assert msg.width == 10000, msg.width
        assert msg.point_step == 32, msg.point_step
        assert msg.row_step == 320000, msg.row_step
        arr = np.frombuffer(msg.data, dtype=np.float32)
        points3d = arr.reshape((msg.height, msg.width, 8))[:, :, 0:3]  # keep only (x, y, z)
        return points3d[:, ::10, :]  # downsample to everh 10th

    def points(self, msg):
        self.points_count += 1
        rospy.loginfo_throttle(10, "points callback: {}".format(self.points_count))
        self.bus.publish('points', self.convert_points(msg))


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("need robot name as argument", file=sys.stderr)
        sys.exit(2)
    try:
        main(sys.argv[1])
    except rospy.exceptions.ROSInterruptException:
        rospy.loginfo("shutdown")

