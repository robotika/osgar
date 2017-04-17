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

from lib.landmarks import ConeLandmarkFinder
from lib.localization import SimpleOdometry

class NearObstacle:
    pass


def min_dist(data):
    data = np.array(data)
    mask = (data > 0)
    if np.any(mask):
        return np.min(data[mask]) * 0.001
    return None 

prev_cones = []
def detect_near_extension(robot, id, data):
    if id=='laser':
        if data is not None and data != []:
            if min_dist(data) < 0.5:
                raise NearObstacle()

            finder = ConeLandmarkFinder()
            global prev_cones
            cones = finder.find_cones(data)
            print '(%.2f, %.2f, %.3f)' % robot.localization.pose(), finder.match_pairs(prev_cones, cones)
            prev_cones = cones
            # TODO:
            #  - collection of all potential cones
            #  - cross distance verification
            #  - "feature tracking"
            #  - localization
            #  - camera verification


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
        can.relog(can_log_name, timestamps_log=open(metalog.getLog('timestamps'), 'w'))
    can.resetModules(configFn=setup_faster_update)
    robot = JohnDeere(can=can, localization=SimpleOdometry())
    robot.UPDATE_TIME_FREQUENCY = 20.0  # TODO change internal and integrate setup

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

