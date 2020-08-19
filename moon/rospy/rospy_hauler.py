#!/usr/bin/python
"""
  Proxy for ROS sensors and effectors
  this is Python 2.7 code
"""

import sys
import rospy

from rospy_rover import RospyRover, RospyRoverReqRep, RospyRoverPushPull
from srcp2_msgs.msg import HaulerMsg

class RospyHaulerPushPull(RospyRoverPushPull):

    def register_handlers(self):
        super(RospyHaulerPushPull, self).register_handlers()
        rospy.Subscriber('/' + self.robot_name + '/bin_info', HaulerMsg, self.callback_topic, '/' + self.robot_name + '/bin_info')

class RospyHaulerReqRep(RospyRoverReqRep):
    pass

class RospyHauler(RospyRover):
    pass

if __name__ == '__main__':
    rh = RospyHauler()
    rh.launch(RospyHaulerPushPull, RospyHaulerReqRep, sys.argv[1:])

# vim: expandtab sw=4 ts=4
