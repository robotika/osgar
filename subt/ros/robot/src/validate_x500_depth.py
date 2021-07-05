#!/usr/bin/env python2
"""
  Validate RGBD data for X500 for up/down camera and pass only valid images
"""

import rospy

from rtabmap_ros.msg import RGBDImage


def validation_callback(msg):
    global valid_rgbd_pub
    rospy.loginfo("X500 validator - received data")
    valid_rgbd_pub.publish(msg)


if __name__ == '__main__':
    try:
        rospy.init_node('validate_x500_depth', log_level=rospy.DEBUG)
        rospy.loginfo("X500 validator started!")
        valid_rgbd_pub = rospy.Publisher('/rgbd_image_in', RGBDImage, queue_size=10)
        rospy.Subscriber('/rgbd_image_out',RGBDImage, validation_callback)
        rospy.loginfo("X500 validator ready to spin!")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
