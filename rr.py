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
from navpat import min_dist
from lib.localization import SimpleOdometry

SAFE_DISTANCE_STOP = 1.0 # meters
SAFE_DISTANCE_GO = SAFE_DISTANCE_STOP + 0.3
SAFE_SIDE_DISTANCE_STOP = 0.6 # meters
SAFE_SIDE_DISTANCE_GO = SAFE_SIDE_DISTANCE_STOP + 0.1
DESIRED_SPEED = 0.5  # m/s

INFINITY = 100.0  # m

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
    robot.set_desired_steering(0.0)  # i.e. go straight (!)

    try:
        robot.extensions.append(('emergency_stop', emergency_stop_extension))
        print robot.canproxy.buttons_and_LEDs
        wait_for_start(robot)
        print robot.canproxy.buttons_and_LEDs

        prev_laser = None
        last_laser_update = None
        moving = False
        dist = None
        distL, distR = None, None
        
        while True:
            robot.update()
            if robot.laser_data is not None:
                assert len(robot.laser_data) == 541, len(robot.laser_data)
                if robot.laser_data != prev_laser:
                    prev_laser = robot.laser_data
                    last_laser_update = robot.time
                    distL = min_dist(robot.laser_data[:180], INFINITY)
                    dist = min_dist(robot.laser_data[180:360], INFINITY)
                    distR = min_dist(robot.laser_data[360:], INFINITY)

            print "dist", distL, dist, distR

            if moving:
                if dist is None or dist < SAFE_DISTANCE_STOP or min(distL, distR) < SAFE_SIDE_DISTANCE_STOP:
                    print "!!! STOP !!!",  distL, dist, distR
                    robot.canproxy.stop()
                    moving = False
            else:  # not moving
                if dist is not None and dist > SAFE_DISTANCE_GO and min(distL, distR) > SAFE_SIDE_DISTANCE_GO:
                    print "GO",  distL, dist, distR
                    robot.set_desired_speed(DESIRED_SPEED)
                    moving = True                

            if last_laser_update is not None and robot.time - last_laser_update > 0.3:
                print "!!!WARNING!!! Missing laser updates for last {:.1f}s".format(robot.time - last_laser_update)
                dist = None  # no longer valid distance measurements
            
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

