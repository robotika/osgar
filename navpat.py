#!/usr/bin/python
"""
  Navigate given pattern in selected area
  usage:
       ./navpat.py <notes> | [<metalog> [<F>]]
"""

import sys
import math
import numpy as np

from apyros.metalog import MetaLog, disableAsserts
from apyros.sourcelogger import SourceLogger

from can import CAN, DummyMemoryLog, ReplayLogInputsOnly, ReplayLog
from johndeere import JohnDeere, setup_faster_update, ENC_SCALE

from driver import go_straight, turn
from helper import attach_sensor, detach_all_sensors


class NearObstacle:
    pass


def min_dist(data):
    data = np.array(data)
    mask = (data > 0)
    if np.any(mask):
        return np.min(data[mask]) * 0.001
    return None 


def detect_near_extension(robot, id, data):
    if id=='laser':
        if data is not None and data != []:
            if min_dist(data) < 0.5:
                raise NearObstacle()

            """
            ZONE_RADIUS = 2.0
            arr = []
            for i in xrange(0, len(data), 10):
                arr.append(min_dist(data[i:i+10]))

            for i in xrange(1, len(arr) - 3):
                if (arr[i] is not None and
                   (arr[i-1] is None or arr[i] < arr[i-1] - ZONE_RADIUS) and
                   (arr[i+1] is None or arr[i] < arr[i+1] - ZONE_RADIUS)):
                    print i, arr[i-1:i+2],
                elif (arr[i] is not None and arr[i+1] is not None and
                     (abs(arr[i] - arr[i+1]) < 0.3) and
                     (arr[i-1] is None or arr[i] < arr[i-1] - ZONE_RADIUS) and
                     (arr[i+2] is None or arr[i] < arr[i+2] - ZONE_RADIUS)):
                    print i, arr[i-1:i+3],
            print
            # TODO:
            #  - collection of all potential cones
            #  - cross distance verification
            #  - "feature tracking"
            #  - localization
            #  - camera verification
            """

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
    for sensor_name in ['gps', 'laser', 'camera']:
        attach_sensor(robot, sensor_name, metalog)

    robot.canproxy.stop()
    robot.canproxy.set_turn_raw(0)

    speed = 0.5

    try:
        robot.extensions.append(('detect_near', detect_near_extension))

        for i in xrange(10):
            go_straight(robot, distance=4.0, speed=speed, with_stop=False)
            turn(robot, math.radians(180), radius=2.0, speed=speed, with_stop=False, timeout=20.0)
        
            # TODO change second radius once the localization & navigation are repeatable
            go_straight(robot, distance=4.0, speed=speed, with_stop=False)
            turn(robot, math.radians(180), radius=2.0, speed=speed, with_stop=False, timeout=20.0)
    except NearObstacle:
        print "Near Exception Raised!"
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

    navigate_pattern(metalog)

# vim: expandtab sw=4 ts=4 

