"""
  Proxy for ROS sensors and effectors
  this is Python 2.7 code
"""

import sys
import rospy
import yaml

from rospy_hauler import RospyHauler, RospyHaulerReqRep, RospyHaulerPushPull
from rospy_round2 import RospyRound2, RospyRound2ReqRep, RospyRound2PushPull

class RospyHaulerRound2PushPull(RospyHaulerPushPull, RospyRound2PushPull):
    pass

class RospyHaulerRound2ReqRep(RospyHaulerReqRep, RospyRound2ReqRep):
    pass

class RospyHaulerRound2(RospyHauler, RospyRound2):
    pass        
        
        
if __name__ == '__main__':
    rs = RospyHaulerRound2()
    rs.launch(RospyHaulerRound2PushPull, RospyHaulerRound2ReqRep, sys.argv[1:])

# vim: expandtab sw=4 ts=4
