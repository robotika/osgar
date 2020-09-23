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
import msgpack

from sensor_msgs.msg import Imu, LaserScan
from nav_msgs.msg import Odometry


def py3round(f):
    if abs(round(f) - f) == 0.5:
        return int(2.0 * round(f / 2.0))
    return int(round(f))


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
        raw = msgpack.packb(data, use_bin_type=True)
        with self.lock:
            self.push.send_multipart([channel, raw])

    def listen(self):
        while not rospy.is_shutdown():
            try:
                channel, bytes_data = self.pull.recv_multipart()
                data = msgpack.unpackb(bytes_data, raw=False)
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
            ('/' + robot_name + '/imu/data', Imu, self.imu, ('rot', 'acc', 'orientation')),
        ]

        # configuration specific topics
        robot_description = rospy.get_param("/{}/robot_description".format(robot_name))
        if "robotika_x2_sensor_config_1" in robot_description:
            rospy.loginfo("robotika x2")
        elif "ssci_x2_sensor_config_1" in robot_description:
            rospy.loginfo("ssci x2")
        elif "ssci_x4_sensor_config_2" in robot_description:
            rospy.loginfo("ssci drone")
            topics.append(('/' + robot_name + '/top_scan', LaserScan, self.top_scan, ('top_scan',)))
            topics.append(('/' + robot_name + '/bottom_scan', LaserScan, self.bottom_scan, ('bottom_scan',)))
            topics.append(('/' + robot_name + '/odom_fused', Odometry, self.odom_fused, ('pose3d',)))
        elif "TeamBase" in robot_description:
            rospy.loginfo("teambase")
        elif "robotika_freyja_sensor_config" in robot_description:
            # possibly fragile detection if freya has bredcrumbs
            rospy.sleep(1)
            publishers, subscribers = rostopic.get_topic_list()
            for name, type, _ in subscribers:
                if name == "/{}/breadcrumb/deploy".format(robot_name):
                    rospy.loginfo("freya 2 (with comms beacons)")
                    break
            else:
                rospy.loginfo("freya 1")
        else:
            rospy.logerror("unknown configuration")
            return

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
            print("_"*50)
            print("receiving:", data)
            # TODO: switch on channel to feed different ROS publishers

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

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("need robot name as argument", file=sys.stderr)
        sys.exit(2)
    main(sys.argv[1])

