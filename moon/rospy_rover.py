#!/usr/bin/python
"""
  Wait for all necessary ROS sensors
"""
import time
import struct
import math
from io import BytesIO
from threading import RLock

import zmq

import rospy
from std_msgs.msg import *  # Float64, JointState
from sensor_msgs.msg import *
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point

# SRCP2 specific
from srcp2_msgs.msg import Qual1ScoringMsg, VolSensorMsg, Qual3ScoringMsg
from srcp2_msgs.srv import (ToggleLightSrv, LocalizationSrv, Qual1ScoreSrv,
                            AprioriLocationSrv, HomeLocationSrv, HomeAlignedSrv)


g_socket = None
g_lock = RLock()


def socket_send(data):
    global g_socket, g_lock
    assert g_socket is not None
    with g_lock:
        g_socket.send(data)


def osgar_debug(msg):
    # send debug data do OSGAR
    socket_send('debug ' + msg)

def callback_lidar(data):
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    # print(rospy.get_caller_id(), data)

    # https://answers.ros.org/question/303115/serialize-ros-message-and-pass-it/
    s1 = BytesIO()
    data.serialize(s1)
    to_send = s1.getvalue()
    header = struct.pack('<I', len(to_send))
    socket_send(header + to_send)


def callback_imu(data):
    s1 = BytesIO()
    data.serialize(s1)
    to_send = s1.getvalue()
    header = struct.pack('<I', len(to_send))
    socket_send(header + to_send)


def callback_topic(data, topic_name):
    s1 = BytesIO()
    data.serialize(s1)
    to_send = s1.getvalue()
    header = struct.pack('<I', len(to_send))
    socket_send(topic_name + '\0' + header + to_send)


def ros2zmq():
    global g_socket, g_lock

    with g_lock:
        context = zmq.Context()
        g_socket = context.socket(zmq.PUSH)
        g_socket.setsockopt(zmq.LINGER, 100)  # milliseconds
        g_socket.bind('tcp://*:5555')

    context2 = zmq.Context()
    g_socket2 = context2.socket(zmq.PULL)
    g_socket2.RCVTIMEO = 5000  # in milliseconds
    g_socket2.bind('tcp://*:5556')
  
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/scout_1/joint_states', JointState, callback_topic, '/scout_1/joint_states')
    rospy.Subscriber('/scout_1/laser/scan', LaserScan, callback_lidar)
    rospy.Subscriber('/scout_1/imu', Imu, callback_imu)

    lights_on = rospy.ServiceProxy('/scout_1/toggle_light', ToggleLightSrv)
    lights_on('high')

    # TODO load it from configuration
    # task 1
    rospy.Subscriber('/qual_1_score', Qual1ScoringMsg, callback_topic, '/qual_1_score')
    rospy.Subscriber('/scout_1/volatile_sensor', VolSensorMsg, callback_topic, '/scout_1/volatile_sensor')

    rospy.Subscriber('/scout_1/camera/left/image_raw/compressed', CompressedImage, callback_topic,
                     '/scout_1/camera/left/image_raw/compressed')
    rospy.Subscriber('/scout_1/camera/right/image_raw/compressed', CompressedImage, callback_topic, 
                     '/scout_1/camera/right/image_raw/compressed')

    steering_msg = Float64()
    steering_msg.data = 0
    effort_msg = Float64()
    effort_msg.data = 0

    QSIZE = 10
    vel_fl_publisher = rospy.Publisher('/scout_1/fl_wheel_controller/command', Float64, queue_size=QSIZE)
    vel_fr_publisher = rospy.Publisher('/scout_1/fr_wheel_controller/command', Float64, queue_size=QSIZE)
    vel_bl_publisher = rospy.Publisher('/scout_1/bl_wheel_controller/command', Float64, queue_size=QSIZE)
    vel_br_publisher = rospy.Publisher('/scout_1/br_wheel_controller/command', Float64, queue_size=QSIZE)

    steering_fl_publisher = rospy.Publisher('/scout_1/fl_steering_arm_controller/command', Float64, queue_size=QSIZE)
    steering_fr_publisher = rospy.Publisher('/scout_1/fr_steering_arm_controller/command', Float64, queue_size=QSIZE)
    steering_bl_publisher = rospy.Publisher('/scout_1/bl_steering_arm_controller/command', Float64, queue_size=QSIZE)
    steering_br_publisher = rospy.Publisher('/scout_1/br_steering_arm_controller/command', Float64, queue_size=QSIZE)

    r = rospy.Rate(100)
    osgar_debug('starting ...')
    while True:
        try:
            message = ""
            try:
                while 1:
                    message = g_socket2.recv(zmq.NOBLOCK)
            except:
                pass

            message_type = message.split(" ")[0]
            if message_type == "cmd_rover":
                arr = [float(x) for x in message.split()[1:]]
                for pub, angle in zip(
                        [steering_fl_publisher, steering_fr_publisher, steering_bl_publisher, steering_br_publisher],
                        arr[:4]):
                    steering_msg.data = angle
                    pub.publish(steering_msg)

                for pub, effort in zip(
                        [vel_fl_publisher, vel_fr_publisher, vel_bl_publisher, vel_br_publisher],
                        arr[4:]):
                    effort_msg.data = effort
                    pub.publish(effort_msg)

            elif message_type == "request_origin":
                osgar_debug('calling request_origin')
                print("Requesting true pose")
                try:
                    rospy.wait_for_service("/scout_1/get_true_pose", timeout=2.0)
                    request_origin = rospy.ServiceProxy('/scout_1/get_true_pose', LocalizationSrv)
                    p = request_origin(True)
                    s = "origin scout_1 %f %f %f  %f %f %f %f" % (p.pose.position.x, p.pose.position.y, p.pose.position.z, 
                                                                  p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w)
                    print(s)
                    socket_send(s)
                except rospy.service.ServiceException as e:
                    print(e)
                    osgar_debug('rospy exception')

            elif message_type == "artf":
                osgar_debug('calling artf')
                s = message.split()[1:]  # ignore "artf" prefix
                x, y, z = [float(a) for a in s[1:]]
                pose = geometry_msgs.msg.Point(x, y, z)
                vol_type = s[0]
                print("Reporting artifact %s at position %f %f %f" % (vol_type, x, y, z))
                try:
                    rospy.wait_for_service("/vol_detected_service", timeout=2.0)
                    report_artf = rospy.ServiceProxy('/vol_detected_service', Qual1ScoreSrv)
                    resp = report_artf(pose=pose, vol_type=vol_type)
                    print("Volatile report result: %r" % resp.result)
                    osgar_debug('volatile result:' + str(resp.result))
                except rospy.ServiceException as exc:
                    print("/vol_detected_service exception: " + str(exc))
                    osgar_debug('volatile exception')

            else:
                if len(message_type) > 0: 
                    print("Unhandled message type: %s" % message_type)

        except zmq.error.Again:
            pass
        r.sleep()


if __name__ == '__main__':
    ros2zmq()

# vim: expandtab sw=4 ts=4
