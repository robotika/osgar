#!/usr/bin/python
"""
  Wait for all necessary ROS sensors
  this is Python 2.7 code
"""
from functools import partial

import rospy
from rospy_base import RospyBase, RospyBaseReqRep, RospyBasePushPull

from geometry_msgs.msg import Point

# SRCP2 specific
from srcp2_msgs.msg import Qual3ScoringMsg
from srcp2_msgs.srv import (AprioriLocationSrv, HomeLocationSrv, HomeAlignedSrv)


class RospyRound3PushPull(RospyBasePushPull):

    def register_handlers(self):
        super(RospyRound3PushPull, self).register_handlers()
        rospy.Subscriber('/qual_3_score', Qual3ScoringMsg, partial(self.callback_topic, rate=50), '/qual_3_score')

class RospyRound3ReqRep(RospyBaseReqRep):

    def process_message(self, message):
        # print("rospy_round3 OSGAR:" + message)
        result = super(RospyRound3ReqRep, self).process_message(message)
        if len(result) == 0:
            message_type = message.split(" ")[0]

            if message_type == "artf":
                s = message.split()[1:]  # ignore "artf" prefix
                vol_type = s[0]
                if vol_type == 'cubesat':
                    # Task 3
                    x, y, z = [float(a) for a in s[1:]]
                    pose = Point(x, y, z)
                    print ("rospy_round3: Reporting %s at position %f %f %f" % (vol_type, x, y, z))
                    try:
                        rospy.wait_for_service('/apriori_location_service', timeout=2.0)
                        report_artf = rospy.ServiceProxy('/apriori_location_service', AprioriLocationSrv)
                        result = report_artf(pose)
                        return 'ok'
                    except rospy.service.ServiceException as e:
                        print("rospy_round3: Apriori position response: %s" % str(e))
                        return str(e)
                    except rospy.ROSException as exc:
                        print("/apriori_location_service not available: " + str(exc))
                        return str(exc)
                elif vol_type == 'homebase':
                    # Task 3
                    print("rospy_round3: reporting homebase arrival")
                    rospy.wait_for_service("/arrived_home_service", timeout=2.0)
                    report_artf = rospy.ServiceProxy('/arrived_home_service', HomeLocationSrv)
                    try:
                        result = report_artf(True)
                        return 'ok'
                        print("rospy_round3: Homebase arrival service response: %r" % result)
                    except rospy.service.ServiceException as e:
                        print("rospy_round3: Homebase arrival service response: Incorrect")
                        return str(e)
                    except rospy.ROSException as exc:
                        print("rospy_round3: /arrived_home_service not available: " + str(exc))
                        return str(exc)
                elif vol_type == 'homebase_alignment':
                    # Task 3
                    rospy.wait_for_service("/aligned_service", timeout=2.0)
                    report_artf = rospy.ServiceProxy('/aligned_service', HomeLocationSrv)
                    try:
                        result = report_artf(True)
                        print("rospy_round3: Aligned service response: %r" % result)
                        return 'ok'
                    except rospy.service.ServiceException as e:
                        print("rospy_round3: Aligned service response: Incorrect")
                        return str(e)
                    except rospy.ROSException as exc:
                        print("rospy_round3: /aligned_service not available: " + str(exc))
                        return str(exc)
                else:
                    return ''
            else:
                return ''
        else:
            return result


class RospyRound3Helper(RospyBase):
    pass

class RospyRound3(RospyRound3Helper):
    pass


# vim: expandtab sw=4 ts=4
