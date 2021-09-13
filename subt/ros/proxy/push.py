#!/usr/bin/python

import math
import numpy as np
import threading
import zmq

import rospy

from osgar.lib.serialize import serialize
from sensor_msgs.msg import LaserScan, PointCloud2


def publish_scan(scan, push, push_lock, channel):
    scan = [(0 if math.isinf(x) else int(1000*x)) for x in scan.ranges]
    raw = serialize(scan)
    with push_lock:
        push.send_multipart([channel, raw])


def publish_pointcloud(points, push, push_lock, channel):
    assert points.height == 1, points.height
    assert points.point_step == 12, points.point_step
    assert points.row_step == points.width * points.point_step, (points.row_step, points.width, points.point_step)
    arr = np.frombuffer(points.data, dtype=np.float32)
    points3d = arr.reshape((points.height, points.width, 3))
    raw = serialize(points3d)
    with push_lock:
        push.send_multipart([channel, raw])


if __name__ == '__main__':
    rospy.init_node('push', log_level=rospy.DEBUG)

    endpoint = rospy.get_param('~endpoint', 'tcp://*:5565')
    scan_channel = rospy.get_param('~output_scan_channel', 'scan')
    pointcloud_channel = rospy.get_param('~output_pointcloud_channel', 'points')

    context = zmq.Context.instance()
    push = context.socket(zmq.PUSH)
    push.LINGER = 100
    push.SNDTIMEO = 1000
    push.bind(endpoint)
    push_lock = threading.Lock()

    scan_subscriber = rospy.Subscriber('scan', LaserScan, lambda scan: publish_scan(scan, push, push_lock, scan_channel))
    pointcloud_subscriber = rospy.Subscriber('points', PointCloud2, lambda points: publish_pointcloud(points, push, push_lock, pointcloud_channel))

    rospy.spin()

    push.close()
