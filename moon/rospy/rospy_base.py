#!/usr/bin/python
"""
  Wait for all necessary ROS sensors
  this is Python 2.7 code
"""
import time
import struct
import math
from io import BytesIO

import threading
from threading import RLock
from threading import Thread

import signal

import zmq
import sys, getopt

import rospy
from rosgraph_msgs.msg import Clock

interrupted = False


def signal_handler(signum, frame):
    global interrupted
    interrupted = True


class RospyBasePushPull(Thread):
    def __init__(self, argv):
        Thread.__init__(self)
        try:
            opts, args = getopt.getopt(argv, '', ['robot_name=', 'push_port=', 'pull_port=', 'reqrep_port='])
        except getopt.GetoptError as e:
            print ("rospy_base.py --push_port<port> --pull_port=<port>; " + str(e))
            sys.exit(2)

        for opt, arg in opts:
            if opt in ['--push_port']:
                self.PUSH_PORT = arg
            elif opt in ['--pull_port']:
                self.PULL_PORT = arg

        self.pull_socket = None

        self.g_socket = None
        self.g_lock = RLock()
        self.prev_time = None

    def setup_sockets(self, context=None):

        with self.g_lock:
            context = context or zmq.Context()
            self.g_socket = context.socket(zmq.PUSH)
            self.g_socket.setsockopt(zmq.LINGER, 0)  # milliseconds
            self.g_socket.bind('tcp://*:' + self.PUSH_PORT)

        context2 = context or zmq.Context()
        self.pull_socket = context2.socket(zmq.PULL)
        self.pull_socket.setsockopt(zmq.LINGER, 0)  # milliseconds
        self.pull_socket.RCVTIMEO = 2000
        self.pull_socket.bind('tcp://*:' + self.PULL_PORT)

    def register_handlers(self):
        rospy.Subscriber('/clock', Clock, self.callback_clock)

    def process_message(self, message):
        pass

    def socket_send(self, data):
        assert self.g_socket is not None
        with self.g_lock:
            self.g_socket.send(data)

    def callback(self, data, topic_name):
        # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        # print(rospy.get_caller_id(), data)

        # https://answers.ros.org/question/303115/serialize-ros-message-and-pass-it/
        s1 = BytesIO()
        data.serialize(s1)
        to_send = s1.getvalue()
        header = struct.pack('<I', len(to_send))
        self.socket_send(topic_name + '\0' + header + to_send)

    def callback_clock(self, data):
        if (self.prev_time is not None and 
                self.prev_time.nsecs//100000000 != data.clock.nsecs//100000000):
            # run at 10Hz, i.e. every 100ms
            s1 = BytesIO()
            data.serialize(s1)
            to_send = s1.getvalue()
            header = struct.pack('<I', len(to_send))
            self.socket_send(header + to_send)
        self.prev_time = data.clock

    def callback_topic(self, data, topic_name):
        s1 = BytesIO()
        data.serialize(s1)
        to_send = s1.getvalue()
        header = struct.pack('<I', len(to_send))
        self.socket_send(topic_name + '\0' + header + to_send)

    def run(self):

        while True:
            try:
                message = self.pull_socket.recv()
                result = self.process_message(message)
            except zmq.Again as e:
                pass

            if interrupted:
                self.pull_socket.close()
                self.g_socket.close()
                break


class RospyBaseReqRep(Thread):
    def __init__(self, argv):
        Thread.__init__(self)
        try:
            opts, args = getopt.getopt(argv, '', ['robot_name=', 'push_port=', 'pull_port=', 'reqrep_port='])
        except getopt.GetoptError:
            print ("rospy_base.py --reqrep_port=<port>")
            sys.exit(2)

        for opt, arg in opts:
            if opt in ['--reqrep_port']:
                self.REQREP_PORT = arg
        self.reqrep_socket = None

    def setup_sockets(self, context=None):
        context2 = context or zmq.Context().instance()
        self.reqrep_socket = context2.socket(zmq.REP)
        self.reqrep_socket.setsockopt(zmq.LINGER, 0)  # milliseconds
        self.reqrep_socket.RCVTIMEO = 2000
        self.reqrep_socket.bind('tcp://127.0.0.1:' + self.REQREP_PORT)

    def process_message(self, message):
        return ''

    def register_handlers(self):
        pass

    def run(self):
        while True:
            try:
                message = self.reqrep_socket.recv()
                response = self.process_message(message)
                self.reqrep_socket.send(response)
            except zmq.Again as e:
                pass
            if interrupted:
                self.reqrep_socket.close()
                break

class RospyBaseHelper(object):

    def launch(self, pushpullclass, reqrepclass, argv):

        signal.signal(signal.SIGTERM, signal_handler) #SIGTERM - kill
        signal.signal(signal.SIGINT, signal_handler) #SIGINT - ctrl-c
        rospy.init_node('listener', anonymous=True)
        context = zmq.Context.instance()

        ppt = pushpullclass(argv)
        ppt.setup_sockets(context)
        ppt.register_handlers()
        ppt.start()

        rrt = reqrepclass(argv)
        rrt.setup_sockets(context)
        rrt.register_handlers()
        rrt.start()

        while True:
            time.sleep(100) # gives room to signal capture
            if interrupted:
                break

        ppt.join()
        rrt.join()

        context.term()

class RospyBase(RospyBaseHelper):
    pass


# vim: expandtab sw=4 ts=4
