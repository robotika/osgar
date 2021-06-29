#!/usr/bin/python
"""
  Wait for all necessary ROS sensors
"""
import time
import struct
from io import BytesIO
import sys
import threading

import zmq
import numpy as np

import rospy
from std_msgs.msg import *
from sensor_msgs.msg import *
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Twist

from sensor_msgs.msg import LaserScan, CompressedImage
from nav_msgs.msg import Odometry


sys.path.append("/home/jetson/ros_ws/src/osgar/osgar/lib")
import serialize as osgar_serialize

ROBOT_NAME = 'X0F200L'
FILTER_ODOM_NTH = 10 #n - every nth message shall be sent to osgar
FILTER_CAMERA_NTH = 4 #n - every nth message shall be sent to osgar
FILTER_DEPTH_NTH = 4 #n - every nth message shall be sent to osgar
g_odom_counter = 0
g_depth_counter = 0
g_camera_counter = 0


class Bus:
    def __init__(self):
        self.lock = threading.Lock()
        context = zmq.Context.instance()
        self.push = context.socket(zmq.PUSH)
        self.push.setsockopt(zmq.LINGER, 100)  # milliseconds
        self.push.bind('tcp://*:5565')
        self.pull = context.socket(zmq.PULL)
        self.pull.LINGER = 100
        self.pull.RCVTIMEO = 100
        self.pull.bind('tcp://*:5566')

    def register(self, *outputs):
        pass

    def publish(self, channel, data):
        raw = osgar_serialize.serialize(data)
        with self.lock:
            self.push.send_multipart([channel, raw])

    def listen(self):
        while not rospy.is_shutdown():
            try:
                channel, bytes_data = self.pull.recv_multipart()
                data = osgar_serialize.deserialize(bytes_data)
                return channel.decode('ascii'), data
            except zmq.ZMQError as e:
                if e.errno != zmq.EAGAIN:
                    rospy.logerr("zmq error")
                    sys.exit("zmq error")
        rospy.loginfo("done")
        sys.exit()



def wait_for_master():
    # it looks like master is not quite ready for several minutes and the only indication is the list of published
    # topics
    rospy.loginfo('wait_for_master')
    while True:
        for topic, topic_type in rospy.get_published_topics():
            if 'battery_state' in topic:
                rospy.loginfo('found ' + topic)
                return topic
        time.sleep(0.1)


def wait_for(topic, topic_type):
    rospy.loginfo('Wait for ' + topic)
    rospy.wait_for_message(topic, topic_type)
    rospy.loginfo('Done with ' + topic)


def wait_for_sensors():
    rospy.init_node('mdwait', anonymous=True)
    wait_for_master()
    rospy.loginfo('-------------- mdwait BEGIN --------------')
    wait_for('/'+ROBOT_NAME+'/imu/data', Imu)
    wait_for('/'+ROBOT_NAME+'/front_scan', LaserScan)
    wait_for('/'+ROBOT_NAME+'/front/image_raw/compressed', CompressedImage)
    wait_for('/'+ROBOT_NAME+'/odom', Odometry)
    wait_for('/'+ROBOT_NAME+'/battery_state', BatteryState)  # note, that this is maybe the critical component!
    rospy.loginfo('--------------- mdwait END ---------------')


g_socket = None

def callback(data):
    global g_socket
    assert g_socket is not None

    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    # print(rospy.get_caller_id(), data)

    # https://answers.ros.org/question/303115/serialize-ros-message-and-pass-it/
    s1 = BytesIO()
    data.serialize(s1)
    to_send = s1.getvalue()
    header = struct.pack('<I', len(to_send))
    g_socket.send(header + to_send)

def callback_odom(data):
    global g_socket, g_odom_counter
    assert g_socket is not None

    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    # print(rospy.get_caller_id(), data)

    # https://answers.ros.org/question/303115/serialize-ros-message-and-pass-it/
    if g_odom_counter >= FILTER_ODOM_NTH:
        s1 = BytesIO()
        data.serialize(s1)
        to_send = s1.getvalue()
        header = struct.pack('<I', len(to_send))
        g_socket.send(header + to_send)
        g_odom_counter = 0
    else:
        g_odom_counter += 1

def callback_depth(data):
    global g_socket, g_depth_counter
    assert g_socket is not None

    # rospy.loginfo(rospy.get_caller_id() + "I heard depth data")
    # print(rospy.get_caller_id(), data)

    # https://answers.ros.org/question/303115/serialize-ros-message-and-pass-it/
    if g_depth_counter >= FILTER_DEPTH_NTH:
        s1 = BytesIO()
        data.serialize(s1)
        to_send = s1.getvalue()
        header = struct.pack('<I', len(to_send))
        g_socket.send("depth" + header + to_send)
        g_depth_counter = 0
    else:
        g_depth_counter += 1

def callback_camera(data):
    global g_socket, g_camera_counter
    assert g_socket is not None

    # rospy.loginfo(rospy.get_caller_id() + "I heard depth data")
    # print(rospy.get_caller_id(), data)

    # https://answers.ros.org/question/303115/serialize-ros-message-and-pass-it/
    if g_camera_counter >= FILTER_CAMERA_NTH:
        s1 = BytesIO()
        data.serialize(s1)
        to_send = s1.getvalue()
        header = struct.pack('<I', len(to_send))
        g_socket.send(header + to_send)
        g_camera_counter = 0
    else:
        g_camera_counter += 1

def callback_clock(data):
    global g_socket
    assert g_socket is not None

    s1 = BytesIO()
    data.serialize(s1)
    to_send = s1.getvalue()
    header = struct.pack('<I', len(to_send))
    g_socket.send(header + to_send)


def odom2zmq():
    global g_socket
    #wait_for_master()

    context = zmq.Context()
    g_socket = context.socket(zmq.PUSH)
    g_socket.setsockopt(zmq.LINGER, 100)  # milliseconds
    g_socket.bind('tcp://*:5555')

    context2 = zmq.Context()
    g_socket2 = context2.socket(zmq.PULL)
    g_socket2.RCVTIMEO = 5000 # in milliseconds
    g_socket2.bind('tcp://*:5556')
  
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/odom', Odometry, callback_odom)
    rospy.Subscriber('/map_scan', LaserScan, callback)
    rospy.Subscriber('/depth_image', Image, callback_depth)
    rospy.Subscriber('/image', CompressedImage, callback_camera)
    rospy.Subscriber('/clock', Clock, callback_clock)
    
    
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0
    
    r = rospy.Rate(10)
    while True:
        try:
            message = ""
            try:
                while 1:
                    message = g_socket2.recv(zmq.NOBLOCK)
            except:
                pass
            #print("OSGAR:" + message)
            if message.split(" ")[0] == "cmd_vel":
                vel_msg.linear.x = float(message.split(" ")[1])
                vel_msg.angular.z = float(message.split(" ")[2])
                velocity_publisher.publish(vel_msg)
        except zmq.error.Again:
            pass
        r.sleep()


class main:
    def __init__(self):
        rospy.init_node('ros2osgar', log_level=rospy.DEBUG)
        self.bus = Bus()

        topics = [
#            ('/odom', Odometry, self.odom, ('battery_state',)),
            ('/map_scan', LaserScan, self.scan, 'scan'),
            ('/depth_image', Image, self.depth, 'depth'),
#            ('/image', CompressedImage, self.image, 'image'),
        ]

        outputs = [t[-1] for t in topics]
        self.bus.register(outputs)

        for name, type, handler, _ in topics:
            rospy.loginfo("waiting for {}".format(name))
            rospy.wait_for_message(name, type)
            setattr(self, handler.__name__+"_count", 0)
            rospy.Subscriber(name, type, handler)

        # main thread receives data from osgar and sends it to ROS
        while True:
            channel, data = self.bus.listen()
            rospy.loginfo("ignoring: {} {}".format(channel, data))

    def scan(self, msg):
        self.scan_count += 1
        rospy.loginfo_throttle(10, "scan callback: {}".format(self.scan_count))
        scan = [int(x * 1000) if msg.range_min < x < msg.range_max else 0 for x in msg.ranges]
        self.bus.publish('scan', scan)

    def convert_depth(self, msg):
        assert msg.encoding == '32FC1', msg.encoding  # unsupported encoding
        # depth is array of floats, OSGAR uses uint16 in millimeters
        # cut min & max (-inf and inf are used for clipping)
        arr = np.frombuffer(msg.data, dtype=np.dtype('f')) * 1000
        arr = np.clip(arr, 1, 0xFFFF)
        arr = np.ndarray.astype(arr, dtype=np.dtype('H'))
        return np.array(arr).reshape((msg.height, msg.width))

    def depth(self, msg):
        self.depth_count += 1
        rospy.loginfo_throttle(10, "depth callback: {}".format(self.depth_count))
        self.bus.publish('depth', self.convert_depth(msg))

    def image(self, msg):
        self.image_count += 1
        rospy.loginfo_throttle(10, "image callback: {}".format(self.image_count))
        self.bus.publish('image', msg.data)


if __name__ == '__main__':
    #wait_for_master()
    #odom2zmq()
    try:
        main()
    except rospy.exceptions.ROSInterruptException:
        rospy.loginfo("shutdown")

# vim: expandtab sw=4 ts=4
