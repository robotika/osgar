#!/usr/bin/env python

import rospy
import pdb
from std_msgs.msg import *
from sensor_msgs.msg import *

robotName = None


def imageCallback(image):
    global destFrameName, imagePublisher
    image.header.frame_id = destFrameName
    imagePublisher.publish(image)

if __name__ == '__main__':
    try:
        #pdb.set_trace()
        rospy.init_node('frame_changer', anonymous=True)
        destTopicName = rospy.get_param('~dest_topic')
        
        destFrameName = rospy.get_param('~dest_frame')
        srcTopicName = rospy.get_param('~src_topic')
        
        imagePublisher = rospy.Publisher(destTopicName, Image )
        rospy.Subscriber(srcTopicName,Image, imageCallback)
        rospy.spin()


    except rospy.ROSInterruptException:
        pass
