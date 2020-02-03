from __future__ import print_function

import datetime
import multiprocessing
import subprocess
import sys

import bcrypt

import rospy
from std_srvs.srv import SetBool

from rosgraph_msgs.msg import Log
from rospy.topics import Publisher, Subscriber

from rospy.impl import rosout

def loadtest(iterations, cpu):
    password = b"loadtest"
    hashed = b"$2b$15$B9jB9E3NZaOZZAmskKpEF.OAmx7DOlJPbNNjOY.TkcNTeKlDtgwnO"
    start_time = datetime.datetime.now()
    rospy.init_node('loadtest'+str(cpu))
    rospy.loginfo("CPU: {cpu} start time: {start_time}".format(**locals()))
    for i in range(iterations):
        ok = bcrypt.checkpw(password, hashed)
        dt = datetime.datetime.now() - start_time
        if ok:
            rospy.loginfo("CPU: {cpu} iteration: {i:2} time: {dt}".format(**locals()))
        else:
            rospy.logwarn("CPU: {cpu} iteration: {i:2} time: {dt} hash failed!".format(**locals()))
    rospy.loginfo("CPU: {} exititing".format(cpu))

def main(iterations):
    rospy.init_node('loadtest', log_level=rospy.DEBUG)

    rospy.loginfo("waiting for rosout logger")
    while rosout._rosout_pub.get_num_connections() == 0:
        rospy.sleep(1)
    #print(rosout._rosout_pub.get_num_connections())

    rospy.loginfo("waiting for clock")
    rospy.sleep(0.0001)
    rospy.loginfo(repr(rospy.get_rostime()))
    rospy.wait_for_service('/subt/start')
    rospy.wait_for_service('/subt/finish')
    subt_start = rospy.ServiceProxy('/subt/start', SetBool)
    ret = subt_start(True)
    rospy.loginfo("/subt/start: {0}".format(ret.success))

    start_time = datetime.datetime.now()
    rospy.loginfo("start time: {start_time}".format(**locals()))
    processes = []
    for i in range(multiprocessing.cpu_count()):
        p = subprocess.Popen([sys.executable, __file__, str(iterations), str(i)])
        processes.append(p)
        rospy.loginfo("started: {} pid {}".format(i, p.pid))

    timeout = 30
    for p in processes:
        rospy.loginfo("joining: pid {} timeout {}".format(p.pid, timeout))
        p.wait()

    dt = datetime.datetime.now() - start_time
    rospy.loginfo("total duration: {dt}".format(**locals()))
    rospy.loginfo("single iteration: {}s".format(round((dt/iterations).total_seconds(), 2)))
    rospy.loginfo("sleeping for {}s sim time".format(iterations))
    rospy.sleep(iterations);
    subt_finish = rospy.ServiceProxy('/subt/finish', SetBool)
    ret = subt_finish(True)
    rospy.loginfo("/subt/finish: {0}".format(ret.success))

if __name__ == "__main__":
    if len(sys.argv) < 3:
        main(int(sys.argv[1]))
    else:
        loadtest(int(sys.argv[1]), int(sys.argv[2]))
