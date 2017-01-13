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


class Driver:
    pass


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
#    wait_for_start(robot)
    robot.desired_speed = 0.5
    start_time = robot.time
    robot.canproxy.set_turn_raw(0)
    robot.canproxy.go()
    start_dist = robot.canproxy.dist_left_raw + robot.canproxy.dist_right_raw
    arr = []
    while robot.time - start_time < 10.0:
        robot.update()
        arr.append(robot.canproxy.gas)
        dist = ENC_SCALE*(robot.canproxy.dist_left_raw + robot.canproxy.dist_right_raw 
                          - start_dist)/2.0
        if dist > 1.0:
            print "Dist OK at {}s".format(robot.time - start_time), sorted(arr)[len(arr)/2]
            break
    print dist
    robot.stop()
    robot.wait(3.0)
    dist = ENC_SCALE*(robot.canproxy.dist_left_raw + robot.canproxy.dist_right_raw 
                      - start_dist)/2.0
    print dist
    print

    robot.canproxy.go_back()
    start_time = robot.time
    start_dist = robot.canproxy.dist_left_raw + robot.canproxy.dist_right_raw
    arr = []
    while robot.time - start_time < 10.0:
        robot.update()
        arr.append(robot.canproxy.gas)
        dist = ENC_SCALE*(robot.canproxy.dist_left_raw + robot.canproxy.dist_right_raw 
                          - start_dist)/2.0
        if dist < -1.0:
            print "Dist back OK at {}s".format(robot.time - start_time), sorted(arr)[len(arr)/2]
            break
    print dist


#        print robot.time, robot.canproxy.gas
#        if not robot.buttonGo:
#            print "STOP!"
#            break
    robot.canproxy.stop_turn()
    robot.stop()
    robot.wait(3.0)
    dist = ENC_SCALE*(robot.canproxy.dist_left_raw + robot.canproxy.dist_right_raw 
                      - start_dist)/2.0
    print dist



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

