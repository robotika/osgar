#!/usr/bin/python
"""
  Demo - follow a wall/obstacles
  (use SICK laser only)
  usage:
       ./demo-wall.py <task-note> [<metalog> [<F>]]
"""

# Plan:
# - inflate all abstacles by desired distance from wall
# - find tangent to inflated object on the LEFT/RIGHT side
# - set stearing angle based on the angle
# - in case of unexpected obstacle in front STOP

import sys
import math

import numpy as np

from can import CAN, DummyMemoryLog, ReplayLogInputsOnly, ReplayLog
from gps import GPS
from gps import DummyGPS as DummySensor  # TODO move to apyros, as mock
from laser import LaserIP
from johndeere import (JohnDeere, center, go, wait_for_start, 
                       setup_faster_update)
from apyros.metalog import MetaLog, disableAsserts
from apyros.sourcelogger import SourceLogger

from route import Convertor, Route
from line import distance


SAFE_DISTANCE_STOP = 1.5  # meters
SAFE_DISTANCE_GO = SAFE_DISTANCE_STOP + 0.5
WALL_DISTANCE = 2.0

STRAIGHT_EPS = math.radians(10)


def gps_data_extension(robot, id, data):
    if id=='gps':
        robot.gps_data = data
        if robot.localization is not None:
            robot.localization.updateGPS(data) 


def laser_data_extension(robot, id, data):
    if id=='laser':
        robot.laser_data = data


def min_dist(data):
    data = np.array(data)
    mask = (data > 0)
    if np.any(mask):
        return np.min(data[mask]) * 0.001
    return None 


def min_dist_arr(data):
    """Fake segments as in demo.py for Velodyne"""
    num = len(data)
    # laser data are anticlockwise -> swap left, right
    return min_dist(data[num/2:]), min_dist(data[:num/2])


def tangent_circle(dist, radius):
    if dist < 2 * radius:
        if dist >= radius:
            return math.asin(radius/float(dist))
        return math.radians(100)
    return None


def follow_wall_angle(laser_data, radius = 2.0):
    max_angle = None
    for i, dist in enumerate(laser_data):
        dist = dist/1000.0
        laser_angle = math.radians((270-i)/2.0)
        angle = tangent_circle(dist, radius)
        if angle is not None:
            max_angle = max(max_angle, laser_angle + angle)
    return max_angle


def demo(metalog):
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

    # GPS
    gps_log_name = metalog.getLog('gps')
    print gps_log_name
    if metalog.replay:
        robot.gps = DummySensor()
        function = SourceLogger(None, gps_log_name).get
    else:
        robot.gps = GPS(verbose=0)
        function = SourceLogger(robot.gps.coord, gps_log_name).get
    robot.gps_data = None
    robot.register_data_source('gps', function, gps_data_extension) 

    # Laser
    laser_log_name = metalog.getLog('laser')
    print laser_log_name
    if metalog.replay:
        robot.laser = DummySensor()
        function = SourceLogger(None, laser_log_name).get
    else:
        robot.laser = LaserIP()
        function = SourceLogger(robot.laser.scan, laser_log_name).get
    robot.laser_data = None
    robot.register_data_source('laser', function, laser_data_extension) 

    robot.gps.start()  # ASAP so we get GPS fix
    robot.laser.start()

    while robot.gas is None:
        robot.update()

    center(robot)
    wait_for_start(robot)

    moving = False
    robot.desired_speed = 0.5
    start_time = robot.time
    prev_gps = robot.gps_data
    prev_destination_dist = None
    while robot.time - start_time < 30*60:  # limit 30 minutes
        robot.update()
        dist = None
        turn_angle = None
        if robot.laser_data is not None:
            assert len(robot.laser_data) == 541,  len(robot.laser_data)
            distL, distR = min_dist_arr(robot.laser_data[200:-200])
            distL = 20.0 if distL is None else distL
            distR = 20.0 if distR is None else distR
            dist = min(distL, distR)
            turn_angle = follow_wall_angle(robot.laser_data)
        if robot.gps_data != prev_gps:
#            print robot.time, robot.gas, robot.gps_data, (distL, distR)
            prev_gps = robot.gps_data
        if moving:
            if dist is None or dist < SAFE_DISTANCE_STOP:
                print "!!! STOP !!!",  dist, (distL, distR)
                robot.canproxy.stop()
                moving = False
        else:  # not moving
            if dist is not None and dist > SAFE_DISTANCE_GO:
                print "GO",  dist
                robot.canproxy.go()
                moving = True
        if turn_angle is not None:
            turn_cmd = max(-100, min(100, math.degrees(turn_angle)))
            robot.canproxy.set_turn_raw(turn_cmd)
        if not robot.buttonGo:
            print "STOP!"
            break
    robot.canproxy.stop_turn()
    center(robot)
    robot.laser.requestStop()
    robot.gps.requestStop()


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

    demo(metalog)

# vim: expandtab sw=4 ts=4 

