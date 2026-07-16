#!/usr/bin/python

from __future__ import print_function
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import robomath
import pdb

TIMEOUT = 2 #s
FWD_SPEED_THRESHOLD = 0.05
DIST_THRESHOLD = 0.05 #m
lastPose = None
currentPose = None
lastFwdSpeed = 0.0
isStuck = False
stuckPose = None
stuckStartTime = None
lastOdomTime = 0

def on_new_cmdvel_callback(data):
    global lastPose,currentPose,isStuck,stuckPose,stuckStartTime, moveBaseClient
    if lastPose == None or currentPose == None:
        return
    fwdSpeed = data.linear.x
    if not isStuck:
        if fwdSpeed > FWD_SPEED_THRESHOLD and robomath.distPose(lastPose,currentPose) < DIST_THRESHOLD:
            #start of stuck
            isStuck = True
            stuckPose = currentPose
            stuckStartTime = rospy.Time.now()
        #print("Not stuck speed=%lf dist=%lf"%(fwdSpeed,robomath.distPose(lastPose,currentPose)))
    else:
        if fwdSpeed > FWD_SPEED_THRESHOLD and robomath.distPose(lastPose,currentPose) < DIST_THRESHOLD:
            stuckDuration = rospy.Time.now() - stuckStartTime
            stuckDuration = stuckDuration.secs
            #print("In stuck, time=%lf, dist=%lf"%(stuckDuration,robomath.distPose(lastPose,currentPose)))
            if stuckDuration > TIMEOUT:
                print("Virtual bumper!")
                moveBaseClient.cancel_all_goals()
                isStuck = False
        else:
            isStuck = False

def on_new_odom_callback(data):
    global lastPose,currentPose,lastOdomTime
    if data.header.stamp.secs > lastOdomTime:
        lastPose = currentPose
        currentPose = data.pose.pose.position
        lastOdomTime = data.header.stamp.secs


rospy.init_node("virtual_bumper",log_level=rospy.DEBUG)

subscriberOdom = rospy.Subscriber("/rs_t265/odom/sample", Odometry, on_new_odom_callback, queue_size=15)
subscriberCmdVel = rospy.Subscriber("/cmd_vel", Twist, on_new_cmdvel_callback, queue_size=15)
moveBaseClient = actionlib.SimpleActionClient('move_base', MoveBaseAction)
moveBaseClient.wait_for_server()
rospy.spin()
