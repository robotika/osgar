#!/usr/bin/python

from __future__ import print_function
import rospy
from nav_msgs.msg import *
from std_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *

import pdb
    
HEIGHT = 1.5
MAX_ANGULAR = 0.7
MAX_VERTICAL = 0.5
PID_P = 10.0
lastScanDown = 0.0
lastScanUp = 0.0
lastOdom = None
validOdom = True

def scan_down_callback(scan):
    global lastScanDown, publisherTwist
    lastScanDown = scan

def scan_up_callback(scan):
    global lastScanUp, publisherTwist
    lastScanUp = scan

def twist_callback(cmd_vel):
    global HEIGHT, lastScanDown, lastScanUp
    
    desiredVel = min(PID_P * abs(HEIGHT - lastScanDown.ranges[0]), MAX_VERTICAL)
    if lastScanDown.ranges[0] > HEIGHT:
        cmd_vel.linear.z = -desiredVel
    else:
        cmd_vel.linear.z = desiredVel
    """
    if lastScanDown.ranges[0] < lastScanUp.ranges[0]:
        cmd_vel.linear.z = 0.1
    else:
        cmd_vel.linear.z = -0.1
    """
    #this is because drone can't handle too aggressive rotation
    if cmd_vel.angular.z > MAX_ANGULAR: 
        cmd_vel.angular.z = MAX_ANGULAR
    elif cmd_vel.angular.z < -MAX_ANGULAR:
        cmd_vel.angular.z = -MAX_ANGULAR

    #if not validOdom:
    #    #visual odometry error -> turn around and try to relocalize
    #    cmd_vel.linear.x = 0
    #    cmd_vel.linear.y = 0
    #    cmd_vel.angular.z = MAX_ANGULAR

    publisherTwist.publish(cmd_vel)

def odom_callback(odom):
    global lastOdom, publisherOdom, validOdom
    if lastOdom is not None and odom.pose.pose.position.x == 0 and odom.pose.pose.position.y == 0:
        #visual odometry error -> don't return empty odometry
        publisherOdom.publish(lastOdom)
        validOdom = False
    else:
        publisherOdom.publish(odom)
        lastOdom = odom
        validOdom = True
        

rospy.init_node("drone_height",log_level=rospy.DEBUG)
lastTime = rospy.Time.now()
subscriberTwist = rospy.Subscriber("/cmd_vel", Twist, twist_callback, queue_size=15)
subscriberScanDown = rospy.Subscriber("/scan_down", LaserScan, scan_down_callback, queue_size=15)
subscriberScanDown = rospy.Subscriber("/scan_up", LaserScan, scan_up_callback, queue_size=15)
subscriberOdom = rospy.Subscriber('/odom', Odometry, odom_callback, queue_size=15)
publisherTwist = rospy.Publisher("/cmd_vel_drone", Twist, queue_size=50)
publisherOdom = rospy.Publisher("/odom_drone", Odometry, queue_size=50)
rospy.spin()

