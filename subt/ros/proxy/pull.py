#!/usr/bin/python

import io
import numpy as np
import sys
import zmq

import cv_bridge
import rospy
import tf2_ros
import tf.transformations

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, Image

from osgar.lib.serialize import deserialize

if __name__ == '__main__':
    rospy.init_node('pull', log_level=rospy.DEBUG)

    endpoint = rospy.get_param('~endpoint', 'tcp://*:5566')
    robot_frame_id = rospy.get_param('~robot_frame_id', 'robot')
    odom_frame_id = rospy.get_param('~odom_frame_id', 'odom')

    streams = rospy.get_param('~streams')
    assert(streams)
    stream_types = {}
    depth_publishers = {}
    camera_info_publishers = {}
    camera_infos = {}
    for stream_config in streams.split(','):
        stream_params = stream_config.split(':')
        stream_name, stream_type = stream_params[:2]
        stream_types[stream_name] = stream_type
        if stream_type == 'depth':
            depth_publishers[stream_name] = rospy.Publisher(stream_name + '/depth', Image, queue_size=5)
            camera_info_publishers[stream_name] = rospy.Publisher(stream_name + '/camera_info', CameraInfo, queue_size=5)
            w, h, cx, cy, fx, fy = stream_params[2:]
            w, h = int(w), int(h)
            cx, cy = float(cx), float(cy)
            fx, fy = float(fx), float(fy)
            camera_info = CameraInfo()
            camera_info.header.frame_id = stream_name
            camera_info.width = w
            camera_info.height = h
            camera_info.distortion_model = 'brown_conrady'
            camera_info.D = (0, 0, 0, 0, 0)
            camera_info.K = (fx, 0, cx,
                             0, fy, cy,
                             0,  0,  1)
            camera_info.P = (fx, 0, cx, 0,
                             0, fy, cy, 0,
                             0, 0, 1, 0)
            camera_infos[stream_name] = camera_info

    context = zmq.Context.instance()
    pull = context.socket(zmq.PULL)
    pull.LINGER = 100
    pull.RCVTIMEO = 100
    pull.bind(endpoint)

    cv_bridge = cv_bridge.CvBridge()
    tf_broadcaster = tf2_ros.TransformBroadcaster()

    while not rospy.is_shutdown():
        try:
            channel, bytes_data = pull.recv_multipart()
            now = rospy.get_rostime()  # Alternatively and perhaps better, we should send timestamps from Osgar. But then we would somehow need to align thee clocks.
            stream_type = stream_types[channel]
            data = deserialize(bytes_data)
            if stream_type == 'depth':
                depth = (data/1000.0).astype(np.float32) 
                depth_msg = cv_bridge.cv2_to_imgmsg(depth, encoding='passthrough')
                depth_msg.header.frame_id = channel
                depth_msg.header.stamp = now
                camera_info = camera_infos[channel]
                camera_info.header.stamp = now
                camera_info_publishers[channel].publish(camera_info)
                depth_publishers[channel].publish(depth_msg)
            elif stream_type == 'pose':
                xyz, quat = data
                t = TransformStamped()
                t.header.stamp = now
                t.header.frame_id = odom_frame_id
                t.child_frame_id = channel
                ((t.transform.translation.x,
                  t.transform.translation.y,
                  t.transform.translation.z),
                 (t.transform.rotation.x,
                  t.transform.rotation.y,
                  t.transform.rotation.z,
                  t.transform.rotation.w)) = xyz, quat
                tf_broadcaster.sendTransform(t)
        except zmq.ZMQError as e:
            if e.errno != zmq.EAGAIN:
                rospy.logerr("zmq error")
                sys.exit("zmq error")

    pull.close()
