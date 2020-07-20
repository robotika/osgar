#!/usr/bin/python
"""
  Wait for all necessary ROS sensors
  this is Python 2.7 code
"""
import time
import struct
import math
from io import BytesIO

from rospy_base import RospyBase, RospyBaseReqRep, RospyBasePushPull

import zmq
import sys, getopt

import rospy
from std_msgs.msg import *  # Float64, JointState
from sensor_msgs.msg import *
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point

# SRCP2 specific

from srcp2_msgs.srv import (ToggleLightSrv, BrakeRoverSrv, LocalizationSrv, ResetModelSrv)


class RospyRoverPushPull(RospyBasePushPull):
    def __init__(self, argv):
        super(RospyRoverPushPull, self).__init__(argv)
        try:
            opts, args = getopt.getopt(argv, '', ['robot_name=', 'push_port=', 'pull_port=', 'reqrep_port='])
        except getopt.GetoptError as e:
            print ("rospy_rover.py --robot_name=<robot name>" + str(e))
            sys.exit(2)
        for opt, arg in opts:
            if opt in ['--robot_name']:
                self.robot_name = arg

        self.WHEEL_SEPARATION_WIDTH = 1.87325  # meters
        self.WHEEL_SEPARATION_HEIGHT = 1.5748  # meters

    def register_handlers(self):
        super(RospyRoverPushPull, self).register_handlers()

        rospy.Subscriber('/' + self.robot_name + '/joint_states', JointState, self.callback_topic, '/' + self.robot_name + '/joint_states')
        rospy.Subscriber('/' + self.robot_name + '/laser/scan', LaserScan, self.callback_topic, '/' + self.robot_name + '/laser/scan')
        rospy.Subscriber('/' + self.robot_name + '/imu', Imu, self.callback_topic, '/' + self.robot_name + '/imu')

        QSIZE = 10

        rospy.Subscriber('/' + self.robot_name + '/camera/left/image_raw/compressed', CompressedImage, self.callback_topic,
                         '/' + self.robot_name + '/camera/left/image_raw/compressed')
        rospy.Subscriber('/' + self.robot_name + '/camera/right/image_raw/compressed', CompressedImage, self.callback_topic,
                         '/' + self.robot_name + '/camera/right/image_raw/compressed')

        self.speed_msg = Float64()
        self.speed_msg.data = 0

        self.steering_msg = Float64()
        self.steering_msg.data = 0
        self.effort_msg = Float64()
        self.effort_msg.data = 0

        # /name/fl_wheel_controller/command
        self.vel_fl_publisher = rospy.Publisher('/' + self.robot_name + '/fl_wheel_controller/command', Float64, queue_size=QSIZE)
        self.vel_fr_publisher = rospy.Publisher('/' + self.robot_name + '/fr_wheel_controller/command', Float64, queue_size=QSIZE)
        self.vel_bl_publisher = rospy.Publisher('/' + self.robot_name + '/bl_wheel_controller/command', Float64, queue_size=QSIZE)
        self.vel_br_publisher = rospy.Publisher('/' + self.robot_name + '/br_wheel_controller/command', Float64, queue_size=QSIZE)

        self.steering_fl_publisher = rospy.Publisher('/' + self.robot_name + '/fl_steering_arm_controller/command', Float64, queue_size=QSIZE)
        self.steering_fr_publisher = rospy.Publisher('/' + self.robot_name + '/fr_steering_arm_controller/command', Float64, queue_size=QSIZE)
        self.steering_bl_publisher = rospy.Publisher('/' + self.robot_name + '/bl_steering_arm_controller/command', Float64, queue_size=QSIZE)
        self.steering_br_publisher = rospy.Publisher('/' + self.robot_name + '/br_steering_arm_controller/command', Float64, queue_size=QSIZE)

        self.light_up_pub = rospy.Publisher('/' + self.robot_name + '/sensor_controller/command', Float64, queue_size=QSIZE, latch=True)
        self.light_up_msg = Float64()

    def process_message(self, message):
        super(RospyRoverPushPull, self).process_message(message)

#        print("OSGAR:" + message)
        message_type = message.split(" ")[0]
        if message_type == "cmd_rover":
            arr = [float(x) for x in message.split()[1:]]
            for pub, angle in zip(
                    [self.steering_fl_publisher, self.steering_fr_publisher, self.steering_bl_publisher, self.steering_br_publisher],
                    arr[:4]):
                self.steering_msg.data = angle
                pub.publish(self.steering_msg)

            for pub, effort in zip(
                    [self.vel_fl_publisher, self.vel_fr_publisher, self.vel_bl_publisher, self.vel_br_publisher],
                    arr[4:]):
                self.effort_msg.data = effort
                pub.publish(self.effort_msg)

        elif message_type == "set_cam_angle":
            self.light_up_msg.data = float(message.split(" ")[1])
            self.light_up_pub.publish(self.light_up_msg)

        elif message_type == "cmd_vel":
            desired_speed = float(message.split(" ")[1])
            desired_angular_speed = float(message.split(" ")[2])

            if abs(desired_speed) < 0.001 and abs(desired_angular_speed) < 0.001:
                self.speed_msg.data = 0
            else:
                self.speed_msg.data = 100  # force to move forward, always TODO PID with scale to Nm
            self.vel_fl_publisher.publish(self.speed_msg)
            self.vel_fr_publisher.publish(self.speed_msg)
            self.vel_bl_publisher.publish(self.speed_msg)
            self.vel_br_publisher.publish(self.speed_msg)

            if abs(desired_speed) > 0.001:
                if abs(desired_angular_speed) > 0.001:
                    # i.e. turn and go
                    radius = desired_speed/desired_angular_speed
                    angle_left = math.atan2(WHEEL_SEPARATION_HEIGHT/2.0, radius - WHEEL_SEPARATION_WIDTH/2.0)
                    angle_right = math.atan2(WHEEL_SEPARATION_HEIGHT/2.0, radius + WHEEL_SEPARATION_WIDTH/2.0)
                else:
                    angle_left = 0.0
                    angle_right = 0.0

                self.steering_msg.data = angle_left
                self.steering_fl_publisher.publish(steering_msg)
                self.steering_msg.data = angle_right
                self.steering_fr_publisher.publish(steering_msg)
                self.steering_msg.data = -angle_left
                self.steering_bl_publisher.publish(steering_msg)
                self.steering_msg.data = -angle_right
                self.steering_br_publisher.publish(steering_msg)
            else:
                pass  # keep steering angles as they are ...
        else:
            # may be picked up by a subclass
            pass


class RospyRoverReqRep(RospyBaseReqRep):
    def __init__(self, argv):
        super(RospyRoverReqRep, self).__init__(argv)
        try:
            opts, args = getopt.getopt(argv, '', ['robot_name=', 'push_port=', 'pull_port=', 'reqrep_port='])
        except getopt.GetoptError as e:
            print ("rospy_rover.py --robot_name=<robot name>" + str(e))
            sys.exit(2)
        for opt, arg in opts:
            if opt in ['--robot_name']:
                self.robot_name = arg


    def process_message(self, message):
        result = super(RospyRoverReqRep, self).process_message(message)
        if len(result) == 0:
            # print("rospy_rover OSGAR:" + message)
            message_type = message.split(" ")[0]
            if message_type == "set_cam_angle":
                angle = float(message.split(" ")[1])
                print ("rospy_rover: Setting cam angle to: %f" % angle)
                self.light_up_msg.data = angle
                self.light_up_pub.publish(self.light_up_msg)
                return 'OK'

            elif message_type == "set_brakes":
                is_on = message.split(" ")[1].startswith("on")
                brake_torque = 100.0 if is_on else 0.0
                print ("rospy_rover: Setting brakes to: %f" % brake_torque)
                self.brakes(brake_torque)
                return 'OK'

            elif message_type == "set_light_intensity":
                light_level = message.split(" ")[1]
                print ("rospy_rover: Setting light intensity to: %s" % light_level)
                self.lights(light_level)
                return 'OK'

            elif message_type == "reset_model":
                print ("rospy_rover: Resetting model")
                self.reset_model(True)
                return 'OK'

            elif message_type == "request_origin":
                print "rospy_rover: Requesting true pose"
                try:
                    rospy.wait_for_service('/' + self.robot_name + '/get_true_pose', timeout=2.0)
                    request_origin = rospy.ServiceProxy('/' + self.robot_name + '/get_true_pose', LocalizationSrv)
                    p = request_origin(True)
                    print("rospy_rover: true pose [%f, %f, %f]  [%f, %f, %f, %f]" % (p.pose.position.x, p.pose.position.y, p.pose.position.z, p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w))
                    s = "origin %f %f %f  %f %f %f %f" % (p.pose.position.x, p.pose.position.y, p.pose.position.z,
                                                                  p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w)
                    return s
                except rospy.service.ServiceException as e:
                    print(e)
                    return str(e)
            else:
                return ''
        else:
            return result

    def register_handlers(self):
        super(RospyRoverReqRep, self).register_handlers()

        QSIZE = 10

        self.lights = rospy.ServiceProxy('/' + self.robot_name + '/toggle_light', ToggleLightSrv)

        self.light_up_pub = rospy.Publisher('/' + self.robot_name + '/sensor_controller/command', Float64, queue_size=QSIZE, latch=True)
        self.light_up_msg = Float64()

        self.brakes = rospy.ServiceProxy('/' + self.robot_name + '/brake_rover', BrakeRoverSrv)

        self.reset_model = rospy.ServiceProxy('/' + self.robot_name + '/reset_model', ResetModelSrv)


class RospyRoverHelper(RospyBase):
    pass

class RospyRover(RospyRoverHelper):
    pass


# vim: expandtab sw=4 ts=4
