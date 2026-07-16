#!/usr/bin/env python2
"""
  Create ROS frame_id "map", which is global world position in DARPA coordinate system
  (shifted and rotated so that (0, 0, 0) is in the middle of the entrance)
"""
from __future__ import print_function
import time
import sys

import rospy
import tf2_ros
import geometry_msgs.msg

from std_msgs.msg import String

from subt_msgs.srv import PoseFromArtifact


def get_origin(robot_name):
    rospy.loginfo('get_origin for ' + robot_name)
    origin_pose = None
    origin_retry_delay = 0.2
    ORIGIN_RETRY_EXPONENTIAL_BACKOFF = 1.3
    MAX_ORIGIN_RETRY_DELAY = 2.0
    ORIGIN_SERVICE_NAME = "/subt/pose_from_artifact_origin"
    rospy.wait_for_service(ORIGIN_SERVICE_NAME)
    origin_service = rospy.ServiceProxy(ORIGIN_SERVICE_NAME, PoseFromArtifact)
    origin_request = String()
    origin_request.data = robot_name
    while origin_pose is None:
        try:
            origin_response = origin_service(origin_request)
            if origin_response.success:
                origin_pose = origin_response.pose.pose
        except rospy.ServiceException:
            rospy.logerr("Failed to get origin. Trying again.")
            time.sleep(origin_retry_delay)
            origin_retry_delay = min(
                MAX_ORIGIN_RETRY_DELAY,
                ORIGIN_RETRY_EXPONENTIAL_BACKOFF * origin_retry_delay)
    return origin_pose


def publish_odom_to_map_tf(origin_pose):
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_transformStamped = geometry_msgs.msg.TransformStamped()

    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = 'map'
    static_transformStamped.child_frame_id = 'odom'

    static_transformStamped.transform.translation = origin_pose.position
    static_transformStamped.transform.rotation = origin_pose.orientation

    broadcaster.sendTransform(static_transformStamped)


if __name__ == '__main__':

    rospy.init_node('map_tf_broadcaster')
    myargv = rospy.myargv(argv=sys.argv)

    if len(myargv) < 2:
        rospy.logerr('Invalid number of parameters\nusage: '
                     './map_tf_broadcaster.py '
                     'robot_name')
        sys.exit(-1)

    origin = get_origin(myargv[1])
    rospy.loginfo(origin)
    publish_odom_to_map_tf(origin)
    rospy.spin()
