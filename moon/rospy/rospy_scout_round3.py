"""
  Proxy for ROS sensors and effectors
  this is Python 2.7 code

  scout enhanced with reporting detection of volatiles (task specific to Round 1 of Qualification)
"""

import sys
import rospy

from rospy_scout import RospyScout, RospyScoutReqRep, RospyScoutPushPull
from rospy_round3 import RospyRound3, RospyRound3ReqRep, RospyRound3PushPull

from srcp2_msgs.msg import Qual3ScoringMsg
from srcp2_msgs.srv import Qual3ScoreSrv
from geometry_msgs.msg import Point

class RospyScoutRound3PushPull(RospyScoutPushPull, RospyRound3PushPull):
    pass

class RospyScoutRound3ReqRep(RospyRound3ReqRep, RospyScoutReqRep):
    pass

class RospyScoutRound3(RospyScout, RospyRound3):
    pass
        
if __name__ == '__main__':
    rs = RospyScoutRound3()
    rs.launch(RospyScoutRound3PushPull, RospyScoutRound3ReqRep, sys.argv[1:])
