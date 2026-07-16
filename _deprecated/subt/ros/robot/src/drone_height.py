#!/usr/bin/python

import rospy
from nav_msgs.msg import *
from std_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *

HEIGHT = 2.0
MAX_ANGULAR = 0.7
MAX_VERTICAL = 0.7
PID_P = 1.0  # 0.5
PID_I = 0.0  # 0.5


class DroneHeightListener:
    #class that listens all topics needed for controlling drone height
    def __init__(self):
        self.lastScanDown = 0.0
        self.lastScanUp = 0.0
        self.started = False
        
        self.subscriberTwist = rospy.Subscriber("/cmd_vel", Twist, self.twistCallback, queue_size=15)
        self.subscriberScanDown = rospy.Subscriber("/scan_down", LaserScan, self.scanDownCallback, queue_size=15)
        self.subscriberScanDown = rospy.Subscriber("/scan_up", LaserScan, self.scanUpCallback, queue_size=15)
        self.publisherTwist = rospy.Publisher("/cmd_vel_drone", Twist, queue_size=50)

        self.accum = 0.0


    def scanDownCallback(self, scan):
        self.lastScanDown = scan.ranges[0]

    def scanUpCallback(self, scan):
        self.lastScanUp = scan.ranges[0]

    def twistCallback(self, cmd_vel):
        if cmd_vel.angular.z != 0 or cmd_vel.linear.x != 0 or cmd_vel.linear.y != 0:
            self.started = True

        if self.started:
            height = min(HEIGHT, (self.lastScanDown + self.lastScanUp) / 2)
            diff = height - self.lastScanDown
            self.accum += diff
            desiredVel = max(-MAX_VERTICAL, min(MAX_VERTICAL, PID_P * diff + PID_I * self.accum))
            cmd_vel.linear.z = desiredVel

            #this is because drone can't handle too aggressive rotation
            if cmd_vel.angular.z > MAX_ANGULAR:
                cmd_vel.angular.z = MAX_ANGULAR
            elif cmd_vel.angular.z < -MAX_ANGULAR:
                cmd_vel.angular.z = -MAX_ANGULAR

            self.publisherTwist.publish(cmd_vel)


if __name__ == "__main__":
    rospy.init_node("drone_height", log_level=rospy.DEBUG)
    droneHeightListener = DroneHeightListener()
    rospy.spin()
