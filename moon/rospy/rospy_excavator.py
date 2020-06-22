"""
  Proxy for ROS sensors and effectors
  this is Python 2.7 code
"""

import sys
import rospy

from std_msgs.msg import *  # Float64, JointState
from srcp2_msgs.msg import ExcavatorMsg

from rospy_rover import RospyRover, RospyRoverReqRep, RospyRoverPushPull

class RospyExcavatorPushPull(RospyRoverPushPull):

    def register_handlers(self):
        super(RospyExcavatorPushPull, self).register_handlers()

        QSIZE = 10

        self.mount_joint_publisher = rospy.Publisher('/' + self.robot_name + '/mount_joint_controller/command', Float64, queue_size=QSIZE)
        self.basearm_joint_publisher = rospy.Publisher('/' + self.robot_name + '/basearm_joint_controller/command', Float64, queue_size=QSIZE)
        self.distalarm_joint_publisher = rospy.Publisher('/' + self.robot_name + '/distalarm_joint_controller/command', Float64, queue_size=QSIZE)
        self.bucket_joint_publisher = rospy.Publisher('/' + self.robot_name + '/bucket_joint_controller/command', Float64, queue_size=QSIZE)


        self.bucket_msg = Float64()
        self.bucket_msg.data = 0

        rospy.Subscriber('/' + self.robot_name + '/bucket_info', ExcavatorMsg, self.callback_topic, '/' + self.robot_name + '/bucket_info')

    def process_message(self, message):
        super(RospyExcavatorPushPull, self).process_message(message)
        message_type = message.split(" ")[0]
        if message_type == "bucket_position":
            mount = float(message.split(" ")[1])
            basearm = float(message.split(" ")[2])
            distalarm = float(message.split(" ")[3])
            bucket = float(message.split(" ")[4])

            for pub, value in zip(
                    [self.mount_joint_publisher, self.basearm_joint_publisher, self.distalarm_joint_publisher, self.bucket_joint_publisher],
                    (mount, basearm, distalarm, bucket)):
                self.bucket_msg.data = value
                pub.publish(self.bucket_msg)



class RospyExcavatorReqRep(RospyRoverReqRep):
    pass

class RospyExcavator(RospyRover):
    pass

if __name__ == '__main__':
    re = RospyExcavator()
    re.launch(RospyExcavatorPushPull, RospyExcavatorReqRep, sys.argv[1:])

# vim: expandtab sw=4 ts=4
