#!/usr/bin/python

from __future__ import print_function
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

import diagnostic_msgs.msg
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

import tf
import angles
import struct
import math
from roboconfig_car import RoboconfigCar 
import pdb
import numpy as np
    
#for anonymous objects
Object = lambda **kwargs: type("Object", (), kwargs)

lastHeading = -1
lastPose = Object
lastPose.x = 0
lastPose.y = 0
lastPose.heading = 0
isPlannerActive = False

def on_new_twist_callback(data):
    global vel_msg
    vel_msg = data

def updateBase(data):
    global lastTime,lastPose,robot
    redSwitch, speeds, heading = robot.update(data)
    currentTime = rospy.Time.now()
    
    dt = (currentTime - lastTime).to_sec()
    # compute odometry in a typical way given the velocities of the robot
    newPose = Object
    newPose.x = lastPose.x + speeds.fwd * math.cos(lastPose.heading) * dt 
    newPose.y = lastPose.y + speeds.fwd * math.sin(lastPose.heading) * dt
    newPose.heading  = heading #lastPose.heading + speeds.ang * dt
                                                                                                                            
    odomQuat = tf.transformations.quaternion_from_euler(0, 0, newPose.heading)
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
    publisherOdom.publish(odom)    
    
    lastTime = currentTime
    lastPose = newPose 
    
    #publish Imu
    imu = Imu()
    imu.header.frame_id = "odom"
    imu.header.stamp = currentTime
    imu.orientation.w = odomQuat[3]
    imu.orientation.x = odomQuat[0]
    imu.orientation.y = odomQuat[1]
    imu.orientation.z = odomQuat[2]

    publisherImu.publish(imu) 
    


    
#def on_new_tank(data):
#    # pre_transmit()
#    dat = cmd_byte_map['tank'] + struct.pack("<ff", -data.left, -data.right)
#    theSerial.write(dat)


robot = RoboconfigCar()
rospy.init_node("base",log_level=rospy.DEBUG)
lastTime = rospy.Time.now()
subscriberTwist = rospy.Subscriber("/cmd_vel_car", Twist, on_new_twist_callback, queue_size=15)
publisherOdom = rospy.Publisher("odom", Odometry, queue_size=50)
publisherImu = rospy.Publisher("imu", Imu, queue_size=50)


vel_msg = Twist()
vel_msg.linear.x = 0
vel_msg.linear.y = 0
vel_msg.linear.z = 0
vel_msg.angular.x = 0
vel_msg.angular.y = 0
vel_msg.angular.z = 0

#updateBase(vel_msg)
#diagnostic_publisher = rospy.Publisher('/diagnostics', Twist, queue_size=10)
#subscriber_tank = rospy.Subscriber("/cmd_vel_tank", rover_drive.msg.Tank, on_new_tank, queue_size=15)
#rospy.spin()
r = rospy.Rate(10) # 5hz
while not rospy.is_shutdown():
    updateBase(vel_msg)
    r.sleep()


