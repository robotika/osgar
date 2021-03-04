#!/usr/bin/env python2

import sys
import os.path
import time

import rospy
import std_msgs.msg

ROSBAG_SIZE_LIMIT = 2 * 1048576000  # 2GB


def main(*args):
    log_filename = args[1]
    rospy.init_node('sendlog', log_level=rospy.DEBUG)
    pub = rospy.Publisher("/robot_data", std_msgs.msg.String, queue_size=1)

    rospy.loginfo("waiting for subscriber on /robot_data")
    while pub.get_num_connections() == 0:
        if rospy.is_shutdown():
            return
        time.sleep(0.1)

    rospy.loginfo("waiting for {}".format(log_filename))
    while not os.path.isfile(log_filename):
        if rospy.is_shutdown():
            return
        time.sleep(0.1)

    rospy.loginfo("opening {}".format(log_filename))
    with open(log_filename, 'rb') as log_file:
        count = 0
        while not rospy.is_shutdown():
            msg = std_msgs.msg.String()
            msg.data = "{}: Hello robot".format(rospy.get_time())
            pub.publish(msg)

            msg.data = log_file.read()
            if log_file.tell() >= ROSBAG_SIZE_LIMIT:
                rospy.logwarn("ROSBAG_SIZE_LIMIT reached, exiting log sending node")
                return
            pub.publish(msg)
            rospy.loginfo_throttle(10, str(count))
            time.sleep(0.1)
            count += 1

    rospy.loginfo("done sending {}".format(log_filename))


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("need log filename as argument")
        raise SystemExit(1)
    main(*sys.argv)

