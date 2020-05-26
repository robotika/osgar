#!/usr/bin/python
"""
  Proxy for ROS sensors and effectors
  this is Python 2.7 code
"""

import sys
import rospy

from rospy_rover import RospyRover, RospyRoverReqRep, RospyRoverPushPull

class RospyHaulerPushPull(RospyRoverPushPull):
    pass

class RospyHaulerReqRep(RospyRoverReqRep):
    pass

class RospyHauler(RospyRover):
    pass

if __name__ == '__main__':
    rh = RospyHauler()
    rh.launch(RospyHaulerPushPull, RospyHaulerReqRep, sys.argv[1:])

# vim: expandtab sw=4 ts=4
