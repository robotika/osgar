#!/usr/bin/python
"""
  Driver for John Deere
  usage:
       ./driver.py <notes> | [<metalog> [<F>]]
"""

import sys
import math

from apyros.metalog import MetaLog, disableAsserts
from apyros.sourcelogger import SourceLogger

from can import CAN, DummyMemoryLog, ReplayLogInputsOnly, ReplayLog
from johndeere import JohnDeere, setup_faster_update, ENC_SCALE


FRONT_REAR_DIST = 1.3
LEFT_WHEEL_DIST_OFFSET = 0.4  # from central axis


class Driver:
    pass


def go_one_meter(robot, gas=None, speed=None, timeout=10.0, with_stop=True):
    """ Drive 1m with given speed or given gas value """
    start_time = robot.time
    if speed is not None:
        assert gas is None  # only one of them has to be set
        robot.set_desired_speed(speed)
    elif gas is not None:
        assert speed is None
        robot.canproxy.cmd = gas
    else:
        assert 0  # one of [gas, speed] has to be set

    start_dist = robot.canproxy.dist_left_raw + robot.canproxy.dist_right_raw
    arr = []
    while robot.time - start_time < timeout:
        robot.update()
        arr.append(robot.canproxy.gas)
        dist = ENC_SCALE*(robot.canproxy.dist_left_raw + robot.canproxy.dist_right_raw 
                          - start_dist)/2.0
        if abs(dist) > 1.0:
            break
    print "Dist OK at {}s".format(robot.time - start_time), sorted(arr)[len(arr)/2]
    print dist
    if with_stop:
        robot.stop()
        robot.wait(3.0)
        dist = ENC_SCALE*(robot.canproxy.dist_left_raw + robot.canproxy.dist_right_raw 
                          - start_dist)/2.0
        print dist
        print


# inspiration from Eduro
#   def turnG(self, angle, angularSpeed = None, radius = 0.0, angleThreshold = math.radians(10),
#             withStop=True, verbose=False ): 
def turn(robot, angle, radius, speed, timeout=10.0, with_stop=True):
    assert radius > 0, radius  # accept only positive radius
    # angle corresponds to final heading change
    if speed < 0:
        # backup - invert steering logic
        angle = -angle

    start_time = robot.time
    if angle > 0:
        # turn left
        base = radius - LEFT_WHEEL_DIST_OFFSET
    else:
        base = radius + LEFT_WHEEL_DIST_OFFSET

    left_radius = math.sqrt(base)
    steering_angle = math.atan2(FRONT_REAR_DIST, base)
    if angle < 0:
        steering_angle = -steering_angle
    print "Steering", math.degrees(steering_angle)
    robot.set_desired_steering(steering_angle)
    robot.set_desired_speed(speed)
    start_left = robot.canproxy.dist_left_raw
    while robot.time - start_time < timeout:
        robot.update()
        dist_left = (robot.canproxy.dist_left_raw - start_left) * ENC_SCALE
        if abs(dist_left) > abs(angle * left_radius):
            break
    if with_stop:
        robot.stop()
        robot.wait(1.0)

#
#                    angle = sensors['steering'] * TURN_SCALE + TURN_ANGLE_OFFSET  # radians
#                    dh = math.sin(angle) * distL / FRONT_REAR_DIST
#                    dist = math.cos(angle) * distL + LEFT_WHEEL_DIST_OFFSET * dh 
# 


def driver_self_test(driver, metalog):
    assert metalog is not None
    can_log_name = metalog.getLog('can')
    if metalog.replay:
        if metalog.areAssertsEnabled():
            can = CAN(ReplayLog(can_log_name), skipInit=True)
        else:
            can = CAN(ReplayLogInputsOnly(can_log_name), skipInit=True)
    else:
        can = CAN()
        can.relog(can_log_name)
    can.resetModules(configFn=setup_faster_update)
    robot = JohnDeere(can=can)
    robot.UPDATE_TIME_FREQUENCY = 20.0  # TODO change internal and integrate setup

    robot.localization = None  # TODO

    robot.canproxy.stop()
    robot.canproxy.set_turn_raw(0)

#    go_one_meter(robot, speed=0.3, with_stop=False)
#    go_one_meter(robot, speed=0.3, with_stop=False)
#    go_one_meter(robot, speed=0.3)

    for i in xrange(3):
        turn(robot, math.radians(-45), radius=2.6, speed=0.5)
        turn(robot, math.radians(-45), radius=2.6, speed=-0.5)

#    go_one_meter(robot, -9000, with_stop=False)
#    go_one_meter(robot, -9000, with_stop=False)
#    go_one_meter(robot, -9000)

    robot.canproxy.stop_turn()
    robot.wait(3.0)


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print __doc__
        sys.exit(2)
    metalog=None
    if 'meta_' in sys.argv[1]:
        metalog = MetaLog(filename=sys.argv[1])
    elif len(sys.argv) > 2:
        metalog = MetaLog(filename=sys.argv[2])
    if len(sys.argv) > 2 and sys.argv[-1] == 'F':
        disableAsserts()
    
    if metalog is None:
        metalog = MetaLog()

    driver_self_test(Driver(), metalog)

# vim: expandtab sw=4 ts=4 

