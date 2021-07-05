#!/usr/bin/env python2
"""
  Validate RGBD data for X500 for up/down camera and pass only valid images
"""

import rospy
import numpy as np
import cv2

from rtabmap_ros.msg import RGBDImage


def decompress(data):
    # RTABMap compresses float32 depth data into RGBA channes of a PNG image.
    # It does not, however, store information about endiannes. All we can do is
    # hope that the current machine and the data origin machine have the same
    # one.
    return cv2.imdecode(np.frombuffer(data, np.uint8),
                        cv2.IMREAD_UNCHANGED).view(dtype=np.float32)[:,:,0]


def validation_callback(msg):
    global valid_rgbd_pub
    rospy.loginfo("X500 validator - received data")
    arr = decompress(msg.depth_compressed.data)
    height, width = arr.shape
    dist = arr[height // 2:, :].min()
    rospy.loginfo("X500 validator - dist = " + str(dist))
    valid_rgbd_pub.publish(msg)


if __name__ == '__main__':
    try:
        rospy.init_node('validate_x500_depth', log_level=rospy.DEBUG)
        rospy.loginfo("X500 validator started!")
        valid_rgbd_pub = rospy.Publisher('/rgbd_image_out', RGBDImage, queue_size=10)
        rospy.Subscriber('/rgbd_image_in',RGBDImage, validation_callback)
        rospy.loginfo("X500 validator ready to spin!")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
