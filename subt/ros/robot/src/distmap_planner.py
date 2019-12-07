#!/usr/bin/env python

import rospy
import sys
import pdb
from nav_msgs.msg import *
from std_msgs.msg import *
from sensor_msgs.msg import *
from tuw_multi_robot_msgs.msg import Graph
import tf
from visualization_msgs.msg import MarkerArray,Marker
import robomath
from geometry_msgs.msg import Twist, Point
import math
import threading
import numpy as np
import cv2
import math
import  scipy.signal 
#import skimage


#for anonymous objects
Object = lambda **kwargs: type("Object", (), kwargs)


DIST_THRESHOLD = 5 #px - this should be converted to meters
DIST_IN_POLAR = 13 #px

SPEED = 0.2
ANG_FACTOR = 0.5
GOAL_REACHED_THRESHOLD = 0.2
CURRENT_VERTEX_THRESHOLD = 0.2
NEAR_VERTEX_THRESHOLD = 1.0
MIN_VERTEX_WIDTH = 0.4
SPOT_TURNING_THRESHOLD = 0.3

lastMap = np.array([],dtype = np.uint8)
lastCostmap = None
markerId = 0
lastPose = None
lastSegments = []

lock = threading.Lock()

def odomCallback(odom):
    global lock, lastPose, baseLinkFrame, transformListerner

    now = rospy.Time.now()
    try:
        transformListener.waitForTransform("map", baseLinkFrame, now, rospy.Duration(4.0))
        (trans, rot) = transformListener.lookupTransform("map", baseLinkFrame, now)
    except:
        print("transform missed")
        return

    lock.acquire()
    lastPose = Object()
    lastPose.x = trans[0]
    lastPose.y = trans[1]
    lastPose.heading = tf.transformations.euler_from_quaternion(rot)[2]
    lock.release()

def mapCallback(costmap):
    global lastMap, lastCostmap
    mapArray = np.reshape(np.array(costmap.data,dtype = np.uint8),(-1,200))
    mapArray = np.flipud(mapArray)
    lastMap = mapArray
    
    lastCostmap = costmap


def getMarkerForVertex(vertex,text,size):
    global markerId
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.type = marker.TEXT_VIEW_FACING
    marker.id = markerId
    markerId += 1
    marker.action = marker.ADD
    marker.scale.x = size
    marker.scale.y = size
    marker.scale.z = 0.2
    marker.color.r = 0
    marker.color.g = 1.0
    marker.color.b = 0
    marker.color.a = 1
    marker.text = text
    marker.lifetime = rospy.Duration(0.2)
    #marker.pose.position.x = vertex.path[0].x + lastMap.info.origin.position.x
    #marker.pose.position.y = vertex.path[0].y + lastMap.info.origin.position.y
    marker.pose.position.x = vertex.x
    marker.pose.position.y = vertex.y
    marker.pose.orientation.x = 0
    marker.pose.orientation.y = 0
    marker.pose.orientation.z = 0
    marker.pose.orientation.w = 0
    return marker

def getMarkerForPose(pose):
    global markerId
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.type = marker.CYLINDER
    marker.id = markerId
    markerId += 1
    marker.action = marker.ADD
    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.scale.z = 0.2
    marker.color.r = 1.0
    marker.color.g = 0
    marker.color.b = 0
    marker.color.a = 1
    marker.text = "Snap"
    marker.lifetime = rospy.Duration(0.2)
    marker.pose.position.x = pose.x
    marker.pose.position.y = pose.y
    marker.pose.orientation.x = 0
    marker.pose.orientation.y = 0
    marker.pose.orientation.z = 0
    marker.pose.orientation.w = 0
    return marker

def getMarkerForLine(pointA,pointB,color):
    global markerId
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.type = marker.LINE_LIST
    marker.id = markerId
    markerId += 1
    marker.action = marker.ADD
    marker.scale.x = 0.05
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = 1
    marker.lifetime = rospy.Duration(0.2)
    point = Point(pointA.x,pointA.y,0)
    marker.points.append(point)
    point = Point(pointB.x,pointB.y,0)
    marker.points.append(point)
    
    return marker


def findNearestVertex(segments, currentPose):
    minDist = 99999999
    minVertex = None
    for vertex in segments:
        dist = robomath.distPose(vertex, currentPose)
        if dist < minDist:
            minDist = dist
            minVertex = vertex
    print("mindist=%f"%(minDist))
    return minVertex
    

def localmax(a):
    begin= np.empty(a.size//2+1,np.int32)
    end  = np.empty(a.size//2+1,np.int32)
    i=k=0
    begin[k]=0
    search_end=True
    while i<a.size-1:
        if a[i]<a[i+1]:
            begin[k]=i+1
            search_end=True
        if search_end and a[i]>a[i+1]:
            end[k]=i
            k+=1
            search_end=False
        i+=1
    if search_end and i>0  : # Final plate if exists
        end[k]=i
        k+=1
    result = list(zip(begin[:k],end[:k]))
    return result

def skeletonize(img):
    '''
    Adapted from:
    https://opencvpython.blogspot.com/2012/05/skeletonization-using-opencv-python.html
    '''
    assert len(img.shape) == 2  #make sure its single channel
    size = np.size(img)
    tenth_size = size/10
    skel = np.zeros(img.shape,np.uint8)

    ret,img = cv2.threshold(img,127,255,0)
    element = cv2.getStructuringElement(cv2.MORPH_CROSS,(3,3))
    done = False

    while( not done):
        eroded = cv2.erode(img,element)
        temp = cv2.dilate(eroded,element)
        temp = cv2.subtract(img,temp)
        skel = cv2.bitwise_or(skel,temp)
        img = eroded.copy()
        zeros = size - cv2.countNonZero(img)
        if zeros==size:
            done = True
    return skel

def stateGenerator():
    global velocityPub, lastSegments, lastPose, markersPub, lastMap, lastCostmap, transformListener
    
    robotPathMap = np.array([[255]*200]*200, dtype = np.uint8)
    plannedGoal = None
    lastLastPose = None
    subPixelDiffX = 0
    subPixelDiffY = 0

    while True:
        #wait for voronoi 
        while len(lastMap) == 0 or not lastPose:
            print("Waiting for map/pose" )
            yield
                
        (trans, rot) = transformListener.lookupTransform(lastCostmap.header.frame_id, "base_link", rospy.Time(0))
        rotationAngle =  tf.transformations.euler_from_quaternion(rot)[2]
        #shift robotPathMap
        lock.acquire()
        
        if lastLastPose <> None:
            rollX = int((lastLastPose.x - lastPose.x) / lastCostmap.info.resolution + subPixelDiffX)
            subPixelDiffX = (lastLastPose.x - lastPose.x) / lastCostmap.info.resolution + subPixelDiffX - rollX
            rollY = int((lastLastPose.y - lastPose.y) / lastCostmap.info.resolution + subPixelDiffY)
            subPixelDiffY = (lastLastPose.y - lastPose.y) / lastCostmap.info.resolution + subPixelDiffY - rollY

            robotPathMap = np.roll(robotPathMap, rollX, axis = 1)
            robotPathMap = np.roll(robotPathMap, -rollY, axis = 0)
        lastLastPose = lastPose
        lock.release()
        binaryMap = np.array(np.where(lastMap == 0, 255, 0),dtype = np.uint8)
        binaryMap = np.minimum(binaryMap,robotPathMap)
        #pdb.set_trace()
        # binaryMap = np.array(np.where(lastMap == 0, 0, 255),dtype = np.uint8)
        #distMap = skeletonize(binaryMap)
        #distMap = skimage.morphology.skeletonize(binaryMap)

        distMap = np.uint8(cv2.distanceTransform(binaryMap, cv2.DIST_L2, 3))
        distMap = cv2.GaussianBlur(distMap,(3,3),0)
        threshMap = np.where(distMap > DIST_THRESHOLD, distMap, 0)

        polarMap = cv2.linearPolar(threshMap,(threshMap.shape[1]/2,threshMap.shape[0]/2), threshMap.shape[0],cv2.WARP_FILL_OUTLIERS)
        polarMap = np.roll(polarMap,-int(polarMap.shape[0]/2 - rotationAngle * polarMap.shape[0]/(2*math.pi)),0)

        #find histogram of distances around the robot 
        histogram = polarMap[:,DIST_IN_POLAR]
        #print(histogram) 
        #find local maxima -> candidate directions
        candidates = localmax(histogram)
        markerArray = MarkerArray()

        #if directionChange < minAngleDist and directionChange > -math.pi * 3/4:
        #                minVertex = vertex
        #                minAngleDist = abs(directionChange)
        minAngleDist = 999999
        minAngle = None
        isFirst = True
        for candidate in candidates:
            angle = (candidate[0] + candidate[1]) / 2 * math.pi * 2 / histogram.shape[0] - math.pi
            print("angle=%lf"%(angle))
            #if angle < minAngleDist and angle > -math.pi * 3/4:
            size = float(histogram[candidate[0]])/10
            pointA = Object()
            lock.acquire()
            pointA.x = lastPose.x + math.cos(angle - lastPose.heading)*size
            pointA.y = lastPose.y - math.sin(angle - lastPose.heading)*size
            if angle > -math.pi * 0.75 and isFirst: #candidate[0] == candidates[0][0]:
                #it is the first candidate
                color = [1.0,0,0]
                isFirst = False
                minAngle = angle
            else:
                color = [0,1.0,0]
                
            marker = getMarkerForLine(pointA,lastPose,color)
            lock.release()
            markerArray.markers.append(marker)
        
        if minAngle <> None:
            cmd_vel = Twist()
            if abs(minAngle) > SPOT_TURNING_THRESHOLD:
                cmd_vel.linear.x = 0
            else:
                cmd_vel.linear.x = SPEED
            angSpeed = -minAngle * ANG_FACTOR
            cmd_vel.angular.z = angSpeed
            velocityPub.publish(cmd_vel)
        
        #mark own path
        robotPathMap[robotPathMap.shape[0]/2][robotPathMap.shape[0]/2] = 0
        

        print("-----")
        markersPub.publish(markerArray)

        FACTOR = 5
        cv2.circle(threshMap,(100,100),DIST_IN_POLAR,(255))
        #cv2.imshow('distMap',cv2.resize(cv2.applyColorMap(threshMap * FACTOR, cv2.COLORMAP_JET),(300,300)))
        cv2.imshow('distMap',cv2.resize(threshMap * FACTOR,(800,800)))
        cv2.imshow('robotPathMap',cv2.resize(robotPathMap * FACTOR,(400,400)))
        cv2.imshow('binaryMap',cv2.resize(binaryMap * FACTOR,(400,400)))
        cv2.imshow('polar',cv2.resize(polarMap * FACTOR,(300,300)))
        cv2.imshow('histogram',cv2.resize(histogram * FACTOR,(30,300)))
        cv2.waitKey(1)
        yield
        continue


        
        while not len(lastSegments) or not lastMap or not lastPose:
            cmd_vel = Twist()
            cmd_vel.angular.z = SPEED
            velocityPub.publish(cmd_vel)
            print("waiting for segments")
            yield

        #if there is no plan, go to the nearest vertex
        if not len(lastSegments):
            print("No last segments")
            continue
        if not plannedGoal:
            print("Searching nearest vertex")
            nearest = findNearestVertex(lastSegments, lastPose)
            if not nearest:
                continue
            plannedGoal = Object()
            plannedGoal.x = nearest.x
            plannedGoal.y = nearest.y
            
        lock.acquire()
        if robomath.distPose(plannedGoal,lastPose) <= GOAL_REACHED_THRESHOLD:
            #goal is reached
            #TODO plan to the next goal
            print("goal reached")
            #find left most vertex
            minAngleDist = 99999
            minVertex = None
            for vertex in lastSegments:
                dist = robomath.distPose(vertex, lastPose)
                if CURRENT_VERTEX_THRESHOLD < dist < NEAR_VERTEX_THRESHOLD:
                    directionChange = robomath.normalizeAngle(math.atan2(vertex.y - lastPose.y, vertex.x - lastPose.x) - lastPose.heading)
                    if directionChange < minAngleDist and directionChange > -math.pi * 3/4:
                        minVertex = vertex
                        minAngleDist = abs(directionChange)

            lock.release()
            print("minAngleDist=%f"%(minAngleDist)) 
            plannedGoal = minVertex
            yield
        else:
            #go to the goal
            print("go to goal")
            markerArray = MarkerArray()
            marker = getMarkerForPose(plannedGoal)
            for vertex in lastSegments:
                markerArray.markers.append(getMarkerForVertex(vertex,vertex.id,0.1))
            markerArray.markers.append(marker)
            markerArray.markers.append(getMarkerForVertex(lastSegments[0],"first vertex",0.5))
            markersPub.publish(markerArray)
            #try:
            #pdb.set_trace()
            directionChange = robomath.normalizeAngle(math.atan2(plannedGoal.y - lastPose.y, plannedGoal.x - lastPose.x) - lastPose.heading)# - math.pi/2)
            lock.release()
            #except:
            #    print(plannedGoal)
            #    print("lastPose %f,%f,%f" % (lastPose.x,lastPose.y, lastPose.heading))
            #    #pdb.set_trace()
            #print("go2waypoint:directionOfWaypoint:",directionChange)
            cmd_vel = Twist()
            if abs(directionChange) > SPOT_TURNING_THRESHOLD:
                cmd_vel.linear.x = 0
            else:
                cmd_vel.linear.x = SPEED
            angSpeed = directionChange * ANG_FACTOR
            cmd_vel.angular.z = angSpeed
            velocityPub.publish(cmd_vel)
            print("go to goal finished")
            yield
            

        

if __name__ == '__main__':
    try:
        rospy.init_node('voronoi_filter', anonymous=True)
        baseLinkFrame = "base_link"
        print("baseLinkFrame:" + str(baseLinkFrame))
        graphPub = rospy.Publisher('/voronoi_filtered', OccupancyGrid)
        markersPub = rospy.Publisher("/voronoi_markers", MarkerArray, queue_size=50)
        velocityPub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/odom', Odometry, odomCallback, queue_size=15)
        rospy.Subscriber('/local_map',OccupancyGrid, mapCallback)
        transformListener = tf.TransformListener()
        r = rospy.Rate(10) # hz
        state = stateGenerator()
        while not rospy.is_shutdown():
            next(state)
            r.sleep()

    except rospy.ROSInterruptException:
        pass
