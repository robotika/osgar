"""
  ROS (Robot Operating System) Proxy
"""

from threading import Thread
import struct
from xmlrpc.client import ServerProxy
from xmlrpc.server import SimpleXMLRPCServer
import math

import socket


from osgar.bus import BusShutdownException

NODE_HOST, NODE_PORT = ('127.0.0.1', 8000)
PUBLISH_PORT = 8123  # can be changed in configuration


ROS_MESSAGE_TYPES = {
    'std_msgs/String': '992ce8a1687cec8c8bd883ec73ca41d1',
    'geometry_msgs/Twist': '9f195f881246fdfa2798d1d3eebca84a',
    'std_msgs/Imu': '6a62c6daae103f4ff57a132d6f95cec2',

    'sensor_msgs/LaserScan': '90c7ef2dc6895d81024acba2ac42f369',
    'sensor_msgs/Image': '060021388200f6f0f447d0fcd9c64743',
    'sensor_msgs/CompressedImage': '8f7a12909da2c9d3332d540a0977563f',
    'nav_msgs/Odometry': 'cd5e73d190d741a2f92e81eda573aca7',
    'sensor_msgs/PointCloud2': '1158d486dd51d683ce2f1be655c3c181',
}


def prefix4BytesLen(s):
    "adding ROS length"
    if type(s) == str:
        s = bytes(s, encoding='ascii')
    return struct.pack("I", len(s)) + s


def packCmdVel(speed, angularSpeed):
    return struct.pack("dddddd", speed, 0, 0, 0, 0, angularSpeed)


def publisherUpdate(caller_id, topic, publishers):
    print("called 'publisherUpdate' with", (caller_id, topic, publishers))
    return (1, "Hi, I am fine", 42) # (code, statusMessage, ignore) 


def requestTopic(caller_id, topic, publishers):
    print("REQ", (caller_id, topic, publishers))
    return 1, "ready on martind-blabla", ['TCPROS', NODE_HOST, PUBLISH_PORT]


class MyXMLRPCServer(Thread):
    def __init__(self, nodeAddrHostPort):
        Thread.__init__(self)
        self.setDaemon(True)
        self.server = SimpleXMLRPCServer( nodeAddrHostPort )
        print("Listening on port %d ..." % nodeAddrHostPort[1])
        self.server.register_function(publisherUpdate, "publisherUpdate")
        self.server.register_function(requestTopic, "requestTopic")
        self.start()

    def run(self):
        self.server.serve_forever()


class ROSProxy(Thread):
    def __init__(self, config, bus):
        global NODE_PORT, PUBLISH_PORT
        Thread.__init__(self)
        self.setDaemon(True)

        self.bus = bus
        self.verbose = False
        self.desired_speed = 0.0
        self.desired_angular_speed = 0.0

        # http://wiki.ros.org/ROS/EnvironmentVariables
        # ROS_MASTER_URI
        # ROS_MASTER_URI is a required setting that tells nodes where they can locate the master.
        # It should be set to the XML-RPC URI of the master.
        self.ros_master_uri = config.get('ros_master_uri', os.environ['ROS_MASTER_URI'])
        self.ros_client_uri = config['ros_client_uri']

        NODE_PORT = config.get('node_port', NODE_PORT)  # workaround for dynamic port assignment
        PUBLISH_PORT = config.get('publish_port', PUBLISH_PORT)

        self.topic = config['topic']
        self.topic_type = config['topic_type']
        self.subscribe_list = config.get('subscribe', [])
        self.caller_id = "/osgar_node"  # do we need this in configuration? (Yes for more ROSProxy nodes)

    def subscribe_topic(self, topic, topic_type, publish_name):
        print('Subscribe', topic)
        code, status_message, publishers = self.master.registerSubscriber(
                self.caller_id, topic, topic_type, self.ros_client_uri)
        assert code == 1, (code, status_message)
        assert len(publishers) == 1, (topic, publishers) # i.e. fails if publisher is not ready now
        print(publishers)

        publisher = ServerProxy(publishers[0])
        code, status_message, protocol_params = publisher.requestTopic(self.caller_id, topic, [["TCPROS"]])
        assert code == 1, (code, status_message)
        assert len(protocol_params) == 3, protocol_params
        print(code, status_message, protocol_params)
        
        # define TCP connection
        # note, that ROS returns name (protocol_params[1]) instead of IP, which is problem to lookup
        #     -> using local IP for now
        self.bus.publish(publish_name + '_addr', ['127.0.0.1', protocol_params[2]])        

        # initialize connection
        header = prefix4BytesLen(
            prefix4BytesLen('callerid=' + self.caller_id) +
            prefix4BytesLen('topic=' + topic) +
            prefix4BytesLen('type=' + topic_type) +
            prefix4BytesLen('md5sum=' + ROS_MESSAGE_TYPES[topic_type])
            )
        self.bus.publish(publish_name, header)

    def run(self):
        self.server = MyXMLRPCServer( (NODE_HOST, NODE_PORT) )
        self.master = ServerProxy(self.ros_master_uri)
        code, status_message, system_state = self.master.getSystemState('/')
        assert code == 1, code
        assert len(system_state) == 3, system_state
        print("Publishers:")
        for s in system_state[0]:
            print(s)

        for topic, topic_type, publish_name in self.subscribe_list:
            self.subscribe_topic(topic, topic_type, publish_name)

        code, status_message, subscribers = self.master.registerPublisher(
                self.caller_id, self.topic, self.topic_type, self.ros_client_uri)

        print("Subscribers:")
        print(subscribers)

        header = prefix4BytesLen(
            prefix4BytesLen('callerid=' + self.caller_id) +
            prefix4BytesLen('topic=' + self.topic) +
            prefix4BytesLen('type=' + self.topic_type) +
            prefix4BytesLen('md5sum=' + ROS_MESSAGE_TYPES[self.topic_type])
            )

        try:
            ready = False
            while True:
                timestamp, channel, data = self.bus.listen()
                if channel != 'tick' and self.verbose:
                    print(timestamp, channel)
                if channel == 'cmd_vel':
                    self.bus.publish('cmd_vel', header)
                    ready = True
                if ready and channel == 'tick':
                    cmd = prefix4BytesLen(packCmdVel(
                        self.desired_speed, self.desired_angular_speed))
                    self.bus.publish('cmd_vel', cmd)
                if channel == "desired_speed":
                    self.desired_speed, self.desired_angular_speed = data[0]/1000.0, math.radians(data[1]/100.0)
        except BusShutdownException:
            pass

    def request_stop(self):
        self.bus.shutdown()

# vim: expandtab sw=4 ts=4
