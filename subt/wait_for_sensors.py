"""
  Wait for all necessary ROS sensors
"""
import time

import rospy
from sensor_msgs.msg import Imu, LaserScan, CompressedImage, BatteryState
from nav_msgs.msg import Odometry


def wait_for_master():
    # it looks like master is not quite ready for several minutes and the only indication is the list of published
    # topics
    rospy.loginfo('wait_for_master')
    while True:
        for topic, topic_type in rospy.get_published_topics():
            if 'battery_state' in topic:
                rospy.loginfo('found ' + topic)
                return topic
        time.sleep(0.1)


def wait_for(topic, topic_type):
    rospy.loginfo('Wait for ' + topic)
    rospy.wait_for_message(topic, topic_type)
    rospy.loginfo('Done with ' + topic)


def wait_for_sensors():
    rospy.init_node('mdwait', anonymous=True)
    wait_for_master()
    rospy.loginfo('-------------- mdwait BEGIN --------------')
    wait_for('/X2/imu/data', Imu)
    wait_for('/X2/front_scan', LaserScan)
    wait_for('/X2/front/image_raw/compressed', CompressedImage)
    wait_for('/X2/odom', Odometry)
    wait_for('/X2/battery_state', BatteryState)  # note, that this is maybe the critical component!
    rospy.loginfo('--------------- mdwait END ---------------')


if __name__ == '__main__':
    wait_for_sensors()

# vim: expandtab sw=4 ts=4
