#!/usr/bin/python
"""
  Wait for all necessary ROS sensors
  this is Python 2.7 code
"""

from functools import partial

import rospy
from rospy_base import RospyBase, RospyBaseReqRep, RospyBasePushPull

# SRCP2 specific
from srcp2_msgs.msg import Qual1ScoringMsg


class RospyRound1PushPull(RospyBasePushPull):

    def register_handlers(self):
        super(RospyRound1PushPull, self).register_handlers()

        rospy.Subscriber('/qual_1_score', Qual1ScoringMsg, partial(self.callback_topic, rate=50), '/qual_1_score')

class RospyRound1ReqRep(RospyBaseReqRep):
    pass

class RospyRound1Helper(RospyBase):
    pass

class RospyRound1(RospyRound1Helper):
    pass


# vim: expandtab sw=4 ts=4
