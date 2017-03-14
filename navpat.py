#!/usr/bin/python
"""
  Navigate given pattern in selected area
  usage:
       ./navpat.py <notes> | [<metalog> [<F>]]
"""

import sys
import math

from apyros.metalog import MetaLog, disableAsserts
from apyros.sourcelogger import SourceLogger

from can import CAN, DummyMemoryLog, ReplayLogInputsOnly, ReplayLog
from johndeere import JohnDeere, setup_faster_update, ENC_SCALE

from driver import go_straight, turn
from helper import attach_sensor, detach_all_sensors

def navigate_pattern(metalog):
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
    for sensor_name in ['gps', 'laser']: # TODO camera
        attach_sensor(robot, sensor_name, metalog)

    robot.canproxy.stop()
    robot.canproxy.set_turn_raw(0)

    speed = 0.5
    for i in xrange(10):
        go_straight(robot, distance=4.0, speed=speed, with_stop=False)
        turn(robot, math.radians(180), radius=2.0, speed=speed, with_stop=False)
        
        # TODO change second radius once the localization & navigation are repeatable
        go_straight(robot, distance=4.0, speed=speed, with_stop=False)
        turn(robot, math.radians(180), radius=2.0, speed=speed, with_stop=False)

    robot.canproxy.stop()
    robot.canproxy.stop_turn()
    robot.wait(3.0)
    
    detach_all(robot)


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

    navigate_pattern(metalog)

# vim: expandtab sw=4 ts=4 

