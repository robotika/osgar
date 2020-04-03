#!/usr/bin/python

from __future__ import print_function
import rospy
from nav_msgs.msg import *
from std_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *

import pdb
import robomath
    
HEIGHT = 1.0
lastScan = 0.0

def scan_callback(scan):
    global lastScan, publisherTwist
    lastScan = scan

def twist_callback(cmd_vel):
    global HEIGHT, lastScan
    
    if lastScan.ranges[0] > HEIGHT:
        cmd_vel.linear.z = -0.1
    else:
        cmd_vel.linear.z = 0.1
    
    publisherTwist.publish(cmd_vel)

rospy.init_node("drone_height",log_level=rospy.DEBUG)
lastTime = rospy.Time.now()
subscriberTwist = rospy.Subscriber("/cmd_vel", Twist, twist_callback, queue_size=15)
subscriberOdom = rospy.Subscriber("/scan", LaserScan, scan_callback, queue_size=15)
publisherTwist = rospy.Publisher("/cmd_vel_drone", Twist, queue_size=50)
rospy.spin()

