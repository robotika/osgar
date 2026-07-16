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
from geometry_msgs.msg import Twist
import math
import threading

#for anonymous objects
Object = lambda **kwargs: type("Object", (), kwargs)

SPEED = 0.2
ANG_FACTOR = 1.0
GOAL_REACHED_THRESHOLD = 0.2
CURRENT_VERTEX_THRESHOLD = 0.2
NEAR_VERTEX_THRESHOLD = 1.0
MIN_VERTEX_WIDTH = 0.4
SPOT_TURNING_THRESHOLD = 0.3

lastMap = None
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
    global lastMap
    lastMap = costmap


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
    

def segmentsCallback(segments):
    global markersPub, baseLinkFrame, transformListener, lastSegments
    newSegments = []
    #pdb.set_trace()
    for vertex in segments.vertices:
        if vertex.width <= MIN_VERTEX_WIDTH:
            continue
        newVertex = Object()
        newVertex.x = vertex.path[0].x + lastMap.info.origin.position.x
        newVertex.y = vertex.path[0].y + lastMap.info.origin.position.y
        newVertex.id = str(vertex.id)
        newSegments.append(newVertex) 
    lastSegments = newSegments

    #print("%f, %f" % (currentPose.x, currentPose.y))
    #markerArray = MarkerArray()
    
    #for vertex in segments.vertices:
    #    markerArray.markers.append(getMarkerForVertex(vertex,vertex.width))
    #nearest = findNearestVertex(segments, currentPose)
    #markerArray.markers.append(getMarkerForVertex(nearest))
    #markerArray.markers.append(getMarkerForPose(currentPose))
    #markersPub.publish(markerArray)

def stateGenerator():
    global velocityPub, lastSegments, lastPose, markersPub, lastMap

    plannedGoal = None
    while True:
        #wait for voronoi 
        while not lastMap or not lastPose:
            print("Waiting for map/pose (%s,%s)" % (lastMap <> None,lastPose <> None))
            yield
        
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
        rospy.Subscriber('/segments', Graph, segmentsCallback, queue_size=15)
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
