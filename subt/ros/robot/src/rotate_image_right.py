#!/usr/bin/python

import numpy as np
import sys

import cv_bridge
import rospy
import tf.transformations
import tf2_ros

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, Image

def rotated_frame_id(original_frame_id):
    return original_frame_id + '/rotated'


class RotateImageRight:
    def __init__(self):
        self.camera_info_subscriber = rospy.Subscriber('input/camera_info', CameraInfo, self.cameraInfoCallback)
        self.camera_info_publisher = rospy.Publisher("output/camera_info", CameraInfo, queue_size=10)

        self.img_subscriber = rospy.Subscriber('input/image', Image, self.imageCallback)
        self.img_publisher = rospy.Publisher("output/image", Image, queue_size=10)

        self.tf_broadcaster = None

        self.cv_bridge = cv_bridge.CvBridge()


    def cameraInfoCallback(self, input_msg):
        if self.tf_broadcaster is None:
            self.setUpTfBroadcaster(input_msg.header.frame_id, input_msg.header.stamp)

        output_msg = CameraInfo()
        output_msg.header = input_msg.header
        output_msg.header.frame_id = rotated_frame_id(input_msg.header.frame_id)
        output_msg.width = input_msg.height
        output_msg.height = input_msg.width
        output_msg.distortion_model = input_msg.distortion_model
        # Unclear what to do with D. Luckily, we work with virtual cameras
        # without distortion.
        output_msg.D = input_msg.D
        output_msg.K[0]= input_msg.K[4]
        output_msg.K[1] = 0
        output_msg.K[2] = input_msg.K[5]
        output_msg.K[3] = 0
        output_msg.K[4] = input_msg.K[0]
        output_msg.K[5] = input_msg.K[2]
        output_msg.K[6] = 0
        output_msg.K[7] = 0
        output_msg.K[8] = 1
        output_msg.R = input_msg.R
        output_msg.P[0] = input_msg.P[5]
        output_msg.P[1] = 0
        output_msg.P[2] = input_msg.P[6]
        output_msg.P[3] = 0 #input_msg.P[7]
        output_msg.P[4] = 0
        output_msg.P[5] = input_msg.P[0]
        output_msg.P[6] = input_msg.P[2]
        output_msg.P[7] = 0 #input_msg.P[3]
        output_msg.P[8] = 0
        output_msg.P[9] = 0
        output_msg.P[10] = 1
        output_msg.P[11] = 0

        # Probably like this? In Virtual, both values are zero.
        output_msg.binning_x = input_msg.binning_y
        output_msg.binning_y = input_msg.binning_x

        output_msg.roi.x_offset = input_msg.roi.y_offset
        output_msg.roi.y_offset = input_msg.roi.x_offset
        output_msg.roi.height = input_msg.roi.width
        output_msg.roi.width = input_msg.roi.height
        output_msg.roi.do_rectify = input_msg.roi.do_rectify

        self.camera_info_publisher.publish(output_msg)


    def imageCallback(self, input_msg):
        if self.tf_broadcaster is None:
            self.setUpTfBroadcaster(input_msg.header.frame_id, input_msg.header.stamp)

        input_img = self.cv_bridge.imgmsg_to_cv2(
                input_msg, desired_encoding='passthrough')
        output_img = np.rot90(input_img, axes=(1, 0))
        output_msg = self.cv_bridge.cv2_to_imgmsg(
                output_img, encoding=input_msg.encoding)
        output_msg.header = input_msg.header
        output_msg.header.frame_id = rotated_frame_id(input_msg.header.frame_id)
        self.img_publisher.publish(output_msg)


    def setUpTfBroadcaster(self, input_frame_id, now):
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster()

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = input_frame_id
        t.child_frame_id = rotated_frame_id(input_frame_id)
        t.transform.rotation.x = 0
        t.transform.rotation.y = 0
        t.transform.rotation.z = -np.sqrt(2) / 2
        t.transform.rotation.w = np.sqrt(2) / 2

        self.tf_broadcaster.sendTransform(t)


if __name__ == "__main__":
    rospy.init_node("rotate_image_right", log_level=rospy.DEBUG)
    overrider = RotateImageRight()
    rospy.spin()
