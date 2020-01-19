#!/usr/bin/env python

import rospy
import pdb
from std_msgs.msg import *
from sensor_msgs.msg import *

robotName = None


def infoCallback(info):
    global destFrameName, infoPublisher
    info.header.frame_id = destFrameName
    infoPublisher.publish(info)

if __name__ == '__main__':
    try:
        #pdb.set_trace()
        rospy.init_node('frame_changer', anonymous=True)
        destTopicName = rospy.get_param('~dest_topic')
        
        destFrameName = rospy.get_param('~dest_frame')
        srcTopicName = rospy.get_param('~src_topic')
        
        infoPublisher = rospy.Publisher(destTopicName, CameraInfo )
        rospy.Subscriber(srcTopicName,CameraInfo, infoCallback)
        rospy.spin()


    except rospy.ROSInterruptException:
        pass
