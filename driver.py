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


def go_straight(robot, distance, gas=None, speed=None, timeout=10.0, with_stop=True):
    """ Drive 'distance' meters with given speed or given gas value """
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

    robot.set_desired_steering(0.0)  # i.e. go straight (!)

    while robot.time - start_time < timeout:
        robot.update()
        arr.append(robot.canproxy.gas)
        dist = ENC_SCALE*(robot.canproxy.dist_left_raw + robot.canproxy.dist_right_raw 
                          - start_dist)/2.0
        if abs(dist) > distance:
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

    left_radius = math.sqrt(base*base + FRONT_REAR_DIST*FRONT_REAR_DIST)
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
            print 'turned distance', dist_left
            break
    if robot.time - start_time >= timeout:
        print "TURN TIMEOUT!", robot.time - start_time

    if with_stop:
        robot.stop()
        robot.wait(1.0)


def normalizeAnglePIPI( angle ):
    while angle < -math.pi:
        angle += 2*math.pi
    while angle > math.pi:
        angle -= 2*math.pi
    return angle 


def follow_line_gen(robot, line, stopDistance=0.0, turnScale=4.0, offsetSpeed=math.radians(20), offsetDistance=0.03):
    """
    line           ... A line to follow.
    stopDistance   ... The robot stops when closer than this to the endpoint. [m]
    turnScale      ... Magic parameter for the rotational speed. [scaling factor]
    offsetSpeed    ... This extra rotational speed is added when the robot is too far from the line. [rad/s]
    offsetDistance ... When the robot is further than this from the line, some extra correction may be needed. [m]
    """
    while line.distanceToFinishLine(robot.localization.pose()) > stopDistance:
        diff = normalizeAnglePIPI(line.angle - robot.localization.pose()[2])
        x, y, a = robot.localization.pose()
        d = 1.3  # FRONT_REAR_DIST
#        print "deg %.1f" %( math.degrees(diff),), 
        signedDistance = line.signedDistance((x+d*math.cos(a), y+d*math.sin(a))) # + self.centerOffset
        if math.fabs( signedDistance ) > offsetDistance:
            step = max(0.0, min(offsetSpeed, offsetSpeed * (abs(signedDistance)-offsetDistance)/offsetDistance ))
            step = step * 0.5  # hack
            if signedDistance < 0:
                diff += step
            else:
                diff -= step
#        turn = restrictedTurn(turnScale * diff)
#        speed = self.restrictedSpeed(turn)
#        yield  speed, turn 
#        print "dist=%0.3f, diff=%.2f" % (signedDistance, math.degrees(diff)), robot.localization.pose()
        yield diff


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

    go_straight(robot, distance=1.0, speed=0.3, with_stop=True)

    for i in xrange(3):
        turn(robot, math.radians(-45), radius=2.6, speed=0.5)
        turn(robot, math.radians(-45), radius=2.6, speed=-0.5)

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

