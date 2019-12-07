#!/usr/bin/python

from __future__ import print_function
import rospy
from nav_msgs.msg import Odometry

from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

import pdb
import robomath
    
FWD_SPEED = 0.3
ANG_SPEED = 0.5
DISTANCE_THRESHOLD = 0.3

status = "PASS_THRU" # "PASS_THRU","BACK_POSITIVE","BACK_NEGATIVE","FORWARD_POSITIVE","FORWARD_NEGATIVE"
odometry = None
speed = None
startPose = None
stopping = False

def on_new_odometry_callback(odom):
    global odometry,speed
    #x=odom
    #pdb.set_trace()
    odometry = odom.pose.pose.position
    speed = odom.twist.twist.linear
	
def on_new_twist_callback(cmd_vel):
    global status,startPose,stopping, odometry,speed, FWD_SPEED, ANG_SPEED, DISTANCE_THRESHOLD
    #print("Status"+status)
    if cmd_vel.linear.x == 0 and cmd_vel.angular.z <> 0:
        #do action
        if status == "PASS_THRU":
            #action is starting now -> remember starting position
            startPose = odometry
            #select back direction based on ang. velocity
            if cmd_vel.angular.z > 0:
                status = "BACK_POSITIVE"
            else:
                status = "BACK_NEGATIVE"
        else:
            #action continues -> am I far enough?
            if robomath.distPose(startPose, odometry) > DISTANCE_THRESHOLD:
                #stop first
                if speed.x <> 0:
                    stopping = True
                else:
                    stopping = False
                    #change direction of action
                    if status == "BACK_POSITIVE":
                        status = "FORWARD_POSITIVE"
                    elif status == "BACK_NEGATIVE":
                        status = "FORWARD_NEGATIVE"
                    elif status == "FORWARD_POSITIVE":
	                status = "BACK_POSITIVE"
                    elif status == "FORWARD_NEGATIVE":
                        status = "BACK_NEGATIVE"
                    startPose = odometry
            else:
                #continue previous action
                pass
            #print("distpose:"+str(robomath.distPose(startPose, odometry)))
	
        #set velocity according to final status
        cmd_vel_car = Twist()
        cmd_vel_car.linear.x = 0
        cmd_vel_car.linear.y = 0
        cmd_vel_car.linear.z = 0
        cmd_vel_car.angular.x = 0
        cmd_vel_car.angular.y = 0
        cmd_vel_car.angular.z = 0
        
        if stopping:
            cmd_vel_car.linear.x = 0
            cmd_vel_car.angular.z = 0
        else:
            if status == "BACK_POSITIVE":
                cmd_vel_car.linear.x = -FWD_SPEED
                cmd_vel_car.angular.z = ANG_SPEED
            elif status == "BACK_NEGATIVE":
                cmd_vel_car.linear.x = -FWD_SPEED
                cmd_vel_car.angular.z = -ANG_SPEED
            elif status == "FORWARD_POSITIVE":
                cmd_vel_car.linear.x = FWD_SPEED
                cmd_vel_car.angular.z = ANG_SPEED
            elif status == "FORWARD_NEGATIVE":
                cmd_vel_car.linear.x = FWD_SPEED
                cmd_vel_car.angular.z = -ANG_SPEED

    else:
        status = "PASS_THRU"
        cmd_vel_car = cmd_vel
		
    publisherTwist.publish(cmd_vel_car)

rospy.init_node("car_rotation",log_level=rospy.DEBUG)
lastTime = rospy.Time.now()
subscriberTwist = rospy.Subscriber("/cmd_vel", Twist, on_new_twist_callback, queue_size=15)
subscriberOdom = rospy.Subscriber("/odom", Odometry, on_new_odometry_callback, queue_size=15)
publisherTwist = rospy.Publisher("/cmd_vel_car", Twist, queue_size=50)

rospy.spin()

