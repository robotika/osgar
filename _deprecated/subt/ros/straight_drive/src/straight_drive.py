#!/usr/bin/python

import rospy
from geometry_msgs.msg import Twist


SPEED = 0.2


rospy.init_node("straight_drive",log_level=rospy.DEBUG)
velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
vel_msg = Twist()
vel_msg.linear.x = SPEED
vel_msg.linear.y = 0
vel_msg.linear.z = 0
vel_msg.angular.x = 0
vel_msg.angular.y = 0
vel_msg.angular.z = 0

r = rospy.Rate(10) # 10hz 
while not rospy.is_shutdown():
    velocity_publisher.publish(vel_msg)
    r.sleep()
