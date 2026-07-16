#!/usr/bin/python2

"""
In order to run on cloudsim, the solution must call `/subt/start` and `/subt/finish` service. If the solution
does not call these, cloudsim sees it as an error and restarts the whole simulation. The function `main` here
provides such a wrapper.
"""

from __future__ import print_function

import errno
import socket
import subprocess
import sys
import time

import rospy
from rospy.impl import rosout
from std_srvs.srv import SetBool

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

def main():
    "Call /subt/start, call sys.argv[1:], call /subt/finish."
    wait_for_master()
    rospy.init_node('cudatest', log_level=rospy.DEBUG)
    rospy.loginfo("waiting for rosout logger")
    # if rosout is not logged, we will loose anything written to it
    while rosout._rosout_pub.get_num_connections() == 0:
        time.sleep(0.2)
    rospy.loginfo("got rosout logger")

    # we need these services
    rospy.loginfo("waiting for /subt/start and /subt/finish")
    rospy.wait_for_service('/subt/start')
    rospy.wait_for_service('/subt/finish')

    # if clock is not ticking, service /subt/start will return with error
    rospy.loginfo("waiting for clock")
    rospy.sleep(2)
    rospy.loginfo(repr(rospy.get_rostime()))

    try:
        subt_start = rospy.ServiceProxy('/subt/start', SetBool)
        ret = subt_start(True)
        rospy.loginfo("/subt/start: {0}".format(ret.success))
        rospy.loginfo("executing {0}".format(sys.argv[1:]))

        with open('/dev/null') as devnul:
            a = subprocess.check_output(sys.argv[1:], stderr=devnul)
            for line in a.splitlines():
                rospy.loginfo(line)

    finally:
        subt_finish = rospy.ServiceProxy('/subt/finish', SetBool)
        ret = subt_finish(True)
        rospy.loginfo("/subt/finish: {0}".format(ret.success))

if __name__ == "__main__":
    main()
