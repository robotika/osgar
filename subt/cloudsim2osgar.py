#!/usr/bin/env python2

import errno
import math
import socket
import sys
import threading
import time

import rospy
import zmq
import msgpack

from sensor_msgs.msg import Imu


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

    def register(self, *outputs):
        pass

    def publish(self, channel, data):
        raw = msgpack.packb(data, use_bin_type=True)
        with self.lock:
            self.push.send_multipart([channel, raw])


class main:
    def __init__(self):
        # get cloudsim ready
        robot_name = sys.argv[1]
        imu_name = '/'+robot_name+'/imu/data'
        wait_for_master()
        rospy.init_node('imu2osgar', log_level=rospy.DEBUG)
        rospy.loginfo("waiting for {}".format(imu_name))
        rospy.wait_for_message(imu_name, Imu)
        rospy.sleep(2)
        self.imu_count = 0

        # start
        self.bus = Bus()
        self.bus.register('imu', 'rot', 'acc', 'orientation')
        rospy.Subscriber(imu_name, Imu, self.imu)
        rospy.spin()

    def imu(self, msg):
        self.imu_count += 1
        rospy.loginfo_throttle(10, "imu callback: {}".format(self.imu_count))
        acc = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
        orientation = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        angular_velocity = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]

        # copy & paste from rosmsg
        q0, q1, q2, q3 = orientation  # quaternion
        x = math.atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2))
        y = math.asin(2 * (q0 * q2 - q3 * q1))
        z = math.atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3))
        rot = [x, y, z]
        # end of copy & paste from rosmsg

        data = [
            msg.header.stamp.to_nsec()/1000000, # time in milliseconds
            orientation,
            angular_velocity,
            acc,
        ]
        self.bus.publish('rot', [py3round(math.degrees(angle) * 100) for angle in rot])
        self.bus.publish('acc', [py3round(x * 1000) for x in acc])
        self.bus.publish('orientation', orientation)
        self.bus.publish('imu', data)


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("need robot name as argument")
        raise SystemExit(1)
    main()

