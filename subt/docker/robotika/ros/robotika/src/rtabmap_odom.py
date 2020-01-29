#!/usr/bin/env python

import rospy
import sys
import pdb
from nav_msgs.msg import *
from std_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *
import cv2
import numpy as np
import math
import tf

frameId = None

def odomCallback(odom):
    global odomPub, frameId
    now = rospy.Time.now()
    try:
        transformListener.waitForTransform("map", frameId, now, rospy.Duration(4.0))
        (trans, rot) = transformListener.lookupTransform("map", frameId, now)
    except:
        print("rtabmap_to_pose: transform missed")
        return
    #pdb.set_trace()
    
    newOdom = Odometry()
    newOdom.header.stamp = rospy.Time.now()
    newOdom.header.frame_id = 'map'
    newOdom.pose.pose = Pose(Point(trans[0], trans[1], trans[2]), Quaternion(rot[0], rot[1], rot[2], rot[3]))
    
    odomPub.publish(newOdom)
    

if __name__ == '__main__':
    try:
        rospy.init_node('rtabmap_pose', anonymous=True)
        frameId = rospy.get_param('~base_link_frame_id')
        odomPub = rospy.Publisher('/rtabmap_odom', Odometry)
        rospy.Subscriber('/odom', Odometry, odomCallback, queue_size=15)
        transformListener = tf.TransformListener()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
