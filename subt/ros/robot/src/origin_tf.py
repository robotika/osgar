#!/usr/bin/env python2

import sys

import rospy
import tf2_ros

from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
from subt_msgs.srv import PoseFromArtifact

if __name__ == '__main__':
    robot_name = rospy.myargv(sys.argv)[1]

    rospy.init_node('origin_tf')

    origin = None
    origin_retry_delay = 0.2
    ORIGIN_RETRY_EXPONENTIAL_BACKOFF = 1.3
    MAX_ORIGIN_RETRY_DELAY = 2.0
    ORIGIN_SERVICE_NAME = "/subt/pose_from_artifact_origin"
    rospy.wait_for_service(ORIGIN_SERVICE_NAME)
    origin_service = rospy.ServiceProxy(ORIGIN_SERVICE_NAME, PoseFromArtifact)
    origin_request = String()
    origin_request.data = robot_name
    while origin is None:
        try:
            origin_response = origin_service(origin_request)
            if origin_response.success:
                origin = origin_response.pose.pose
        except rospy.ServiceException:
            rospy.logerr("Failed to get origin. Trying again.")
            time.sleep(origin_retry_delay)
            origin_retry_delay = min(
                    MAX_ORIGIN_RETRY_DELAY,
                    ORIGIN_RETRY_EXPONENTIAL_BACKOFF * origin_retry_delay)

    tf_broadcaster = tf2_ros.StaticTransformBroadcaster()
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "global"
    t.child_frame_id = "odom"
    t.transform.translation = origin.position
    t.transform.rotation = origin.orientation

    tf_broadcaster.sendTransform(t)

    rospy.spin()

