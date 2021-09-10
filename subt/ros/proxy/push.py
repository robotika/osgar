#!/usr/bin/python

import math
import zmq

import rospy

from osgar.lib.serialize import serialize
from sensor_msgs.msg import LaserScan

def publish_scan(scan, push, channel):
    scan = [(0 if math.isinf(x) else int(1000*x)) for x in scan.ranges]
    raw = serialize(scan)
    push.send_multipart([channel, raw])

if __name__ == '__main__':
    rospy.init_node('push', log_level=rospy.DEBUG)

    endpoint = rospy.get_param('~endpoint', 'tcp://*:5565')
    scan_channel = rospy.get_param('~output_scan_channel', 'scan')

    context = zmq.Context.instance()
    push = context.socket(zmq.PUSH)
    push.LINGER = 100
    push.SNDTIMEO = 1000
    push.bind(endpoint)

    scan_subscriber = rospy.Subscriber('scan', LaserScan, lambda scan: publish_scan(scan, push, scan_channel))

    rospy.spin()

    push.close()
