#!/usr/bin/python2

from __future__ import print_function

import errno
import socket
import subprocess
import time

import rospy
from rospy.impl import rosout
from std_srvs.srv import SetBool

def wait_for_master():
    print("Waiting for rosmaster")
    start = time.time()
    last_params = []
    while not rospy.is_shutdown():
        try:
            params = rospy.get_param_names()
            if params != last_params:
                print(time.time()-start, params)
                if '/robot_names' in params:
                    return True
                last_params = params
            time.sleep(0.01)
        except socket.error as serr:
            if serr.errno != errno.ECONNREFUSED:
                raise serr  # re-raise error if its not the one we want
            time.sleep(0.5)

def main():
    wait_for_master()
    rospy.init_node('cudatest', log_level=rospy.DEBUG)
    rospy.loginfo("waiting for rosout logger")
    while rosout._rosout_pub.get_num_connections() == 0:
        time.sleep(0.2)
    rospy.loginfo("got rosout logger")

    rospy.loginfo("waiting for clock")
    rospy.sleep(2)
    rospy.loginfo(repr(rospy.get_rostime()))
    rospy.wait_for_service('/subt/start')
    rospy.wait_for_service('/subt/finish')

    try:
        subt_start = rospy.ServiceProxy('/subt/start', SetBool)
        ret = subt_start(True)
        rospy.loginfo("/subt/start: {0}".format(ret.success))

        with open('/dev/null') as devnul:
            a = subprocess.check_output(["/osgar-ws/env/bin/python", "src/cudatest3.py"], stderr=devnul)
            for line in a.splitlines():
                rospy.loginfo(line)

    finally:
        rospy.sleep(2)
        subt_finish = rospy.ServiceProxy('/subt/finish', SetBool)
        ret = subt_finish(True)
        rospy.loginfo("/subt/finish: {0}".format(ret.success))

if __name__ == "__main__":
    main()
