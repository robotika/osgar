#!/usr/bin/python

from __future__ import print_function
import rospy
from nav_msgs.msg import Odometry

import diagnostic_msgs.msg
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

import tf
import angles
import struct
import math
from roboconfig_mob import RoboconfigMob 
import pdb
import numpy as np
    
#for anonymous objects
Object = lambda **kwargs: type("Object", (), kwargs)

WATCHDOG = 1.0 #s

lastHeading = -1
lastPose = Object
lastPose.x = 0
lastPose.y = 0
lastPose.heading = 0
isPlannerActive = False

def on_new_twist_callback(data):
    global vel_msg, lastVelMsgTime
    vel_msg = data
    lastVelMsgTime = rospy.Time.now()

def updateBase(vel_msg):
    global lastTime,lastPose,robot, lastVelMsgTime
    
    velMsgStep = (rospy.Time.now() - lastVelMsgTime).to_sec()
    if velMsgStep > WATCHDOG:
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0

        #print("WATCHDOG: %lf" % (velMsgStep))
    
    redSwitch, speeds = robot.update(vel_msg)
    currentTime = rospy.Time.now()

    dt = (currentTime - lastTime).to_sec()
    # compute odometry in a typical way given the velocities of the robot
    newPose = Object
    newPose.x = lastPose.x + speeds.fwd * math.cos(lastPose.heading) * dt 
    newPose.y = lastPose.y + speeds.fwd * math.sin(lastPose.heading) * dt
    newPose.heading = lastPose.heading + speeds.ang * dt

                                                                                                                            
    odomQuat = tf.transformations.quaternion_from_euler(0, 0, newPose.heading)
    """
    broadcasterOdom.sendTransform(
        (newPose.x, newPose.y, 0.),
        odomQuat,
        currentTime,
        "base_link",
        "odom"
    )
    """
    odom = Odometry()
    odom.header.stamp = currentTime
    odom.header.frame_id = "odom"

    # set the position
    odom.pose.pose = Pose(Point(newPose.x, newPose.y, 0.), Quaternion(*odomQuat))

    # set the velocity
    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(speeds.fwd, 0, 0), Vector3(0, 0, speeds.ang))
    odom.pose.covariance = np.diag([0.01, 0.01, 9999, 9999, 9999, 0.01]).ravel()#x,y,z,rotX,rotY,rotZ
    odom.twist.covariance = np.diag([0.01, 0.01, 9999, 9999, 9999, 0.01]).ravel()
    # publish the message
    publisherWheelOdom.publish(odom)    
    
    lastTime = currentTime
    lastPose = newPose 


robot = RoboconfigMob()
rospy.init_node("base",log_level=rospy.DEBUG)
lastTime = rospy.Time.now()
subscriberTwist = rospy.Subscriber("/cmd_vel", Twist, on_new_twist_callback, queue_size=15)
publisherWheelOdom = rospy.Publisher("wheel_odom", Odometry, queue_size=50)
broadcasterOdom = tf.TransformBroadcaster()

vel_msg = Twist()
vel_msg.linear.x = 0
vel_msg.linear.y = 0
vel_msg.linear.z = 0
vel_msg.angular.x = 0
vel_msg.angular.y = 0
vel_msg.angular.z = 0
lastVelMsgTime = rospy.Time.now()

#updateBase(vel_msg)
#diagnostic_publisher = rospy.Publisher('/diagnostics', Twist, queue_size=10)
#subscriber_tank = rospy.Subscriber("/cmd_vel_tank", rover_drive.msg.Tank, on_new_tank, queue_size=15)
#rospy.spin()
r = rospy.Rate(10)
while not rospy.is_shutdown():
    updateBase(vel_msg)
    r.sleep()


