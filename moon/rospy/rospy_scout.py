"""
  Proxy for ROS sensors and effectors
  this is Python 2.7 code
"""

import sys
import rospy

from rospy_rover import RospyRover, RospyRoverReqRep, RospyRoverPushPull
from srcp2_msgs.msg import VolSensorMsg


class RospyScoutPushPull(RospyRoverPushPull):
    def __init__(self, argv):
        super(RospyScoutPushPull, self).__init__(argv)
        
    def register_handlers(self):
        super(RospyScoutPushPull, self).register_handlers()

        rospy.Subscriber('/' + self.robot_name + '/volatile_sensor', VolSensorMsg, self.callback_topic, '/' + self.robot_name + '/volatile_sensor')

class RospyScoutReqRep(RospyRoverReqRep):
    pass

class RospyScout(RospyRover):
    pass
        
if __name__ == '__main__':
    rs = RospyScout()
    rs.launch(RospyScoutPushPull, RospyScoutReqRep, sys.argv[1:])
