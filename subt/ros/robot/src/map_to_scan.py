#!/usr/bin/env python

import rospy
import sys
import pdb
from nav_msgs.msg import *
from std_msgs.msg import *
from sensor_msgs.msg import *
import cv2
import numpy as np
import math
import tf

def mapCallback(costmap):
    global scan_pub, transformListener
    #pdb.set_trace()
    mapArray = np.reshape(np.array(costmap.data,dtype = np.uint8),(-1,200))
    polar = cv2.linearPolar(mapArray,(mapArray.shape[1]/2,mapArray.shape[0]/2), mapArray.shape[0],cv2.WARP_FILL_OUTLIERS)
    #get rotation between local map and robot base
    (trans, rot) = transformListener.lookupTransform(costmap.header.frame_id, "base_link", rospy.Time(0))
    rotationAngle =  tf.transformations.euler_from_quaternion(rot)[2]
    
    polar = np.roll(polar,-int(polar.shape[0]/2 + rotationAngle * polar.shape[0]/(2*math.pi)),0) 
    #cv2.imshow('Map',cv2.resize(mapArray,(300,300))) 
    scan = LaserScan()
    scan.header.stamp = rospy.Time.now()
    scan.header.frame_id = "base_link/front_laser"
    scan.angle_min = -math.pi #* 3/4
    scan.angle_max = math.pi #* 3/4
    scan.angle_increment = 2 * math.pi / polar.shape[0]   #should be multiplied by 3/2?
    scan.time_increment = 0.0
    scan.range_min = 0.0
    scan.range_max = 10.0
    scan.ranges = np.argmax(polar > 0,axis=1) * costmap.info.resolution
    scan.intensities = []
    scan_pub.publish(scan)

    #cv2.imshow('Polar',cv2.resize(polar,(300,300)))
    #cv2.waitKey(1)

if __name__ == '__main__':
    try:
        rospy.init_node('map_to_scan', anonymous=True)
        scan_pub = rospy.Publisher('/map_scan', LaserScan,queue_size=10)
        rospy.Subscriber('/move_base/local_costmap/costmap',OccupancyGrid, mapCallback)
        transformListener = tf.TransformListener()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
