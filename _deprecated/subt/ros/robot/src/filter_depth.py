#!/usr/bin/python

import numpy as np
import sys

import cv_bridge
import rospy
from sensor_msgs.msg import Image

class FilterDepth:
    def __init__(self, min_depth, max_depth):
        self.min_depth = min_depth
        self.max_depth = max_depth

        self.subscriber = rospy.Subscriber('input', Image, self.depthCallback)
        self.publisher = rospy.Publisher("output", Image, queue_size=10)

        self.cv_bridge = cv_bridge.CvBridge()


    def depthCallback(self, input_msg):
        input_depth = self.cv_bridge.imgmsg_to_cv2(
                input_msg, desired_encoding='passthrough')
        output_depth = np.where(
                np.logical_and(input_depth >= self.min_depth,
                               input_depth <= self.max_depth),
                input_depth,
                np.inf)
        output_msg = self.cv_bridge.cv2_to_imgmsg(
                output_depth, encoding="passthrough")
        output_msg.header = input_msg.header
        self.publisher.publish(output_msg)


if __name__ == "__main__":
    rospy.init_node("filter_depth", log_level=rospy.DEBUG)
    overrider = FilterDepth(*[float(limit) for limit in rospy.myargv(sys.argv)[1:3]])
    rospy.spin()
