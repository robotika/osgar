"""
  Proxy for ROS sensors and effectors
  this is Python 2.7 code
"""

import sys
import rospy
import yaml

from rospy_base import RospyBase, RospyBaseReqRep, RospyBasePushPull
from srcp2_msgs.srv import Qual2VolatilesSrv
from srcp2_msgs.msg import Qual2ScoringMsg

class RospyRound2PushPull(RospyBasePushPull):

    def register_handlers(self):
        super(RospyRound2PushPull, self).register_handlers()
        rospy.Subscriber('/qual_2_score', Qual2ScoringMsg, self.callback_topic, '/qual_2_score')


class RospyRound2ReqRep(RospyBaseReqRep):

    def register_handlers(self):
        super(RospyRound2ReqRep, self).register_handlers()
        self.get_volatile_list = rospy.ServiceProxy('/qual_2_services/volatile_locations', Qual2VolatilesSrv)

    def process_message(self, message):
        result = super(RospyRound2ReqRep, self).process_message(message)
        if len(result) == 0:
            message_type = message.split(" ")[0]
            if message_type == "get_volatile_locations":
                result = self.get_volatile_list()
                vol_list = yaml.safe_load(str(result))
                return ','.join((str(e['x']) + " " + str(e['y'])) for e in vol_list['poses'])
            else:
                return ''
        else:
            return result
        
class RospyRound2(RospyBase):
    pass        
       

# vim: expandtab sw=4 ts=4
