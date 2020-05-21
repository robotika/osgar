#!/usr/bin/python

import rospy
from nav_msgs.msg import *
from std_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *

HEIGHT = 1.5
MAX_ANGULAR = 0.7
MAX_VERTICAL = 0.5
PID_P = 10.0


class DroneHeightListener:
    #class that listens all topics needed for controlling drone height
    def __init__(self):
        self.lastScanDown = 0.0
        self.lastScanUp = 0.0
        self.lastOdom = None
        self.validOdom = True
        
        self.subscriberTwist = rospy.Subscriber("/cmd_vel", Twist, self.twistCallback, queue_size=15)
        self.subscriberScanDown = rospy.Subscriber("/scan_down", LaserScan, self.scanDownCallback, queue_size=15)
        self.subscriberScanDown = rospy.Subscriber("/scan_up", LaserScan, self.scanUpCallback, queue_size=15)
        self.subscriberOdom = rospy.Subscriber('/odom', Odometry, self.odomCallback, queue_size=15)
        self.publisherTwist = rospy.Publisher("/cmd_vel_drone", Twist, queue_size=50)
        self.publisherOdom = rospy.Publisher("/odom_drone", Odometry, queue_size=50)


    def scanDownCallback(self, scan):
        self.lastScanDown = scan

    def scanUpCallback(self, scan):
        self.lastScanUp = scan

    def twistCallback(self, cmd_vel):
        desiredVel = min(PID_P * abs(HEIGHT - self.lastScanDown.ranges[0]), MAX_VERTICAL)
        if self.lastScanDown.ranges[0] > HEIGHT:
            cmd_vel.linear.z = -desiredVel
        else:
            cmd_vel.linear.z = desiredVel
            
        #this is because drone can't handle too aggressive rotation
        if cmd_vel.angular.z > MAX_ANGULAR: 
            cmd_vel.angular.z = MAX_ANGULAR
        elif cmd_vel.angular.z < -MAX_ANGULAR:
            cmd_vel.angular.z = -MAX_ANGULAR

        self.publisherTwist.publish(cmd_vel)

    def odomCallback(self, odom):
        if self.lastOdom is not None and odom.pose.pose.position.x == 0 and odom.pose.pose.position.y == 0:
            #visual odometry error -> don't return empty odometry
            publisherOdom.publish(self.lastOdom)
        else:
            publisherOdom.publish(odom)
            self.lastOdom = odom
        

rospy.init_node("drone_height",log_level=rospy.DEBUG)
droneHeightListener = DroneHeightListener()
rospy.spin()
