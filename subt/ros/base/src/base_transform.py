#!/usr/bin/python

from __future__ import print_function
import rospy
from nav_msgs.msg import Odometry
import tf
import pdb
    

def on_new_odom_callback(odom):
    pose = odom.pose.pose.position
    orientation = odom.pose.pose.orientation
    broadcasterOdom.sendTransform(
        (pose.x,pose.y,pose.z),
        (orientation.x,orientation.y,orientation.z,orientation.w),
        rospy.Time.now(),
        "base_link",
        "odom"
    )
    


rospy.init_node("base_transform",log_level=rospy.DEBUG)
lastTime = rospy.Time.now()
subscriberOdom = rospy.Subscriber("/odom", Odometry, on_new_odom_callback, queue_size=15)
broadcasterOdom = tf.TransformBroadcaster()


r = rospy.spin()


