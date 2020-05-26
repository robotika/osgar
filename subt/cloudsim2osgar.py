#!/usr/bin/env python2

import errno
import socket
import sys
import threading
import time

import rospy
import zmq
import msgpack

from sensor_msgs.msg import Imu

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
        self.lock = threading.Lock()
        context = zmq.Context.instance()
        self.push = context.socket(zmq.PUSH)
        self.push.setsockopt(zmq.LINGER, 100)  # milliseconds
        self.push.bind('tcp://*:5565')
        rospy.Subscriber(imu_name, Imu, self.imu)
        rospy.spin()

    def imu(self, msg):
        self.imu_count += 1
        rospy.loginfo_throttle(60, "imu callback: {}".format(self.imu_count))
        data = [
            msg.header.stamp.to_nsec()/1000, # time in microseconds
            [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w],
            [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z],
            [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z],
        ]
        self.send('imu', data)

    def send(self, channel, data):
        raw = msgpack.packb(data, use_bin_type=True)
        with self.lock:
            self.push.send_multipart([channel, raw])



if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("need robot name as argument")
        raise SystemExit(1)
    main()

