"""
  Proxy for ROS sensors and effectors
  this is Python 2.7 code
"""

import sys
import rospy
import yaml

from rospy_excavator import RospyExcavator, RospyExcavatorReqRep, RospyExcavatorPushPull
from rospy_round2 import RospyRound2, RospyRound2ReqRep, RospyRound2PushPull

from srcp2_msgs.srv import Qual2VolatilesSrv

class RospyExcavatorRound2PushPull(RospyExcavatorPushPull, RospyRound2PushPull):
    pass

class RospyExcavatorRound2ReqRep(RospyExcavatorReqRep, RospyRound2ReqRep):
    pass

class RospyExcavatorRound2(RospyExcavator, RospyRound2):
    pass        
        
if __name__ == '__main__':
    rs = RospyExcavatorRound2()
    rs.launch(RospyExcavatorRound2PushPull, RospyExcavatorRound2ReqRep, sys.argv[1:])

# vim: expandtab sw=4 ts=4
