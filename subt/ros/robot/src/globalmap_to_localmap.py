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
SUBMAP_RADIUS = 600
lastPose = None

def mapCallback(costmap):
    global lastMap
    lastMap = costmap


def odomCallback(odom):
    global lastPose
    lastPose = odom.pose.pose.position

def updateMap():
    global map_pub, lastMap, lastPose, transformListener
    now = rospy.Time.now()
    try:
        transformListener.waitForTransform("map", "base_link", now, rospy.Duration(4.0))
        (trans, rot) = transformListener.lookupTransform("map", "base_link", now)
    except:
        print("map_to_scan: transform missed")
        return
    #pdb.set_trace()
    pose = trans 
    rotationAngle = tf.transformations.euler_from_quaternion(rot)[2]
    mapArray = np.reshape(np.array(lastMap.data,dtype = np.int8),(lastMap.info.height,lastMap.info.width))
    
    #enlarge array so that the subMap works well also at the border of the global map
    mapArray = np.pad(mapArray, pad_width=SUBMAP_RADIUS, mode='constant', constant_values=-1)

    mapY = int((pose[0] - lastMap.info.origin.position.x) / lastMap.info.resolution) + SUBMAP_RADIUS
    mapX = int((pose[1] - lastMap.info.origin.position.y) / lastMap.info.resolution) + SUBMAP_RADIUS
    #pdb.set_trace()
    subMap = mapArray[mapX - SUBMAP_RADIUS:mapX + SUBMAP_RADIUS,mapY - SUBMAP_RADIUS:mapY + SUBMAP_RADIUS]
    #pdb.set_trace()
    #subMap = np.array(subMap > 60,dtype = np.uint8) * 100
    subMap = np.where(subMap > 70,100,subMap)
    #cv2.imshow('subMap',cv2.resize(subMapVisible,(400,400)))
    #cv2.imshow('orig submap', subMap)
    #cv2.waitKey(1)
    #pdb.set_trace()
    
    newMap = OccupancyGrid()
    newMap.header.stamp = rospy.Time.now()
    newMap.header.frame_id = 'odom'
    newMap.info.resolution = lastMap.info.resolution
    newMap.info.width = subMap.shape[0]
    newMap.info.height = subMap.shape[1]
    newMap.info.origin.position.x = lastPose.x - subMap.shape[0] / 2 * lastMap.info.resolution
    newMap.info.origin.position.y = lastPose.y - subMap.shape[1] / 2 * lastMap.info.resolution
    newMap.info.origin.orientation.w = 1.0
    newMap.data = subMap.flatten().tolist()
    map_pub.publish(newMap)
    
   
    #cv2.imshow('Polar',cv2.resize(polar,(300,300)))
    #cv2.waitKey(1)

if __name__ == '__main__':
    try:
        rospy.init_node('map_to_scan', anonymous=True)
        map_pub = rospy.Publisher('/map_out', OccupancyGrid)
        rospy.Subscriber('/map_in',OccupancyGrid, mapCallback)
        rospy.Subscriber('/odom', Odometry, odomCallback, queue_size=15)
        transformListener = tf.TransformListener()
        r = rospy.Rate(20) # 5hz
        while not rospy.is_shutdown():
            if lastMap:
                updateMap()
            r.sleep()


    except rospy.ROSInterruptException:
        pass
