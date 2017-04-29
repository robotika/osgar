#!/usr/bin/python
"""
  Robotem Rovne (Robot go straight) contest with John Deere
  usage:
       ./rr.py <task> [<metalog> [<F>]]
"""
import sys

from apyros.metalog import MetaLog, disableAsserts
from can import CAN, DummyMemoryLog, ReplayLogInputsOnly, ReplayLog
from johndeere import (JohnDeere, setup_faster_update, wait_for_start,
                       emergency_stop_extension, EmergencyStopException)
from driver import go_straight
from helper import attach_sensor, detach_all_sensors
from navpat import NearObstacle, detect_near_extension
from lib.localization import SimpleOdometry


def robot_go_straight(metalog):
    assert metalog is not None
    can_log_name = metalog.getLog('can')
    if metalog.replay:
        if metalog.areAssertsEnabled():
            can = CAN(ReplayLog(can_log_name), skipInit=True)
        else:
            can = CAN(ReplayLogInputsOnly(can_log_name), skipInit=True)
    else:
        can = CAN()
        can.relog(can_log_name, timestamps_log=open(metalog.getLog('timestamps'), 'w'))
    can.resetModules(configFn=setup_faster_update)
    robot = JohnDeere(can=can, localization=SimpleOdometry())
    robot.UPDATE_TIME_FREQUENCY = 20.0  # TODO change internal and integrate setup

    for sensor_name in ['gps', 'laser', 'camera']:
        attach_sensor(robot, sensor_name, metalog)

    robot.canproxy.stop()
    robot.canproxy.set_turn_raw(0)

    print robot.canproxy.buttons_and_LEDs
    wait_for_start(robot)
    print robot.canproxy.buttons_and_LEDs
    speed = 0.5

    try:
        robot.extensions.append(('detect_near', detect_near_extension))
        robot.extensions.append(('emergency_stop', emergency_stop_extension))
        go_straight(robot, distance=400.0, speed=speed, with_stop=False, timeout=3600.0)
    except NearObstacle:
        print "Near Exception Raised!"
        robot.extensions = []  # hack
    except EmergencyStopException:
        print "Emergency STOP Exception!"
        robot.extensions = []  # hack

    robot.canproxy.stop()
    robot.canproxy.stop_turn()
    robot.wait(3.0)
    
    detach_all_sensors(robot)


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

    robot_go_straight(metalog)

# vim: expandtab sw=4 ts=4 

