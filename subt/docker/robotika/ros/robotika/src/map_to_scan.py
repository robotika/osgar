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

lastMap = None
robotName = None
SUBMAP_RADIUS = 100

def mapCallback(costmap):
    global lastMap
    lastMap = costmap


def updateScan():
    global scan_pub, transformListener, lastMap, robotName
    now = rospy.Time.now()
    try:
        transformListener.waitForTransform("map", robotName, now, rospy.Duration(4.0))
        (trans, rot) = transformListener.lookupTransform("map", robotName, now)
    except:
        print("map_to_scan: transform missed")
        return
    #pdb.set_trace()
    pose = trans 
    rotationAngle = tf.transformations.euler_from_quaternion(rot)[2]
    mapArray = np.reshape(np.array(lastMap.data,dtype = np.uint8),(lastMap.info.height,lastMap.info.width)) + 1
    
    #enlarge array so that the subMap works well also at the border of the global map
    mapArray = np.pad(mapArray, pad_width=SUBMAP_RADIUS, mode='constant', constant_values=0)

    mapY = int((pose[0] - lastMap.info.origin.position.x) / lastMap.info.resolution) + SUBMAP_RADIUS
    mapX = int((pose[1] - lastMap.info.origin.position.y) / lastMap.info.resolution) + SUBMAP_RADIUS
    #pdb.set_trace()
    subMap = mapArray[mapX - SUBMAP_RADIUS:mapX + SUBMAP_RADIUS,mapY - SUBMAP_RADIUS:mapY + SUBMAP_RADIUS]
    #pdb.set_trace()
    subMap = np.array(subMap > 60,dtype = np.uint8) * 255
    #cv2.imshow('subMap',cv2.resize(subMap,(400,400)))
    #cv2.imshow('orig submap', subMap)
    #cv2.waitKey(1)
    
    polar = cv2.linearPolar(subMap,(subMap.shape[1]/2,subMap.shape[0]/2), subMap.shape[0],cv2.WARP_FILL_OUTLIERS)
    #get rotation between local map and robot base
    
    polar = np.roll(polar,-int(polar.shape[0]/2 + rotationAngle * polar.shape[0]/(2*math.pi)),0) 
    #cv2.imshow('Map',cv2.resize(mapArray,(300,300))) 
    scan = LaserScan()
    scan.header.stamp = rospy.Time.now()
    scan.header.frame_id = robotName #"base_link/front_laser"
    scan.angle_min = -math.pi #* 3/4
    scan.angle_max = math.pi #* 3/4
    scan.angle_increment = 2 * math.pi / polar.shape[0]   #should be multiplied by 3/2?
    scan.time_increment = 0.0
    scan.range_min = 0.0
    scan.range_max = 10.0
    scan.ranges = np.argmax(polar > 0,axis=1) * lastMap.info.resolution
    scan.intensities = []
    scan_pub.publish(scan)
    #pdb.set_trace()
    
    
    #cv2.imshow('Polar',cv2.resize(polar,(300,300)))
    #cv2.waitKey(1)

if __name__ == '__main__':
    try:
        rospy.init_node('map_to_scan', anonymous=True)
        robotName = rospy.get_param('robot_name')
        scan_pub = rospy.Publisher('/' + robotName + '/map_scan', LaserScan )
        rospy.Subscriber('/rtabmap/grid_prob_map',OccupancyGrid, mapCallback)
        transformListener = tf.TransformListener()
        r = rospy.Rate(20) # 5hz
        while not rospy.is_shutdown():
            if lastMap:
                updateScan()
            r.sleep()


    except rospy.ROSInterruptException:
        pass
