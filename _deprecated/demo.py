#!/usr/bin/python
"""
  Demo - drive and avoid obstacles
  usage:
       ./demo.py <task-note> [<metalog> [<F>]]
"""
import sys
import math
from can import CAN, DummyMemoryLog, ReplayLogInputsOnly, ReplayLog
from gps import GPS
from gps import DummyGPS as DummySensor  # TODO move to apyros, as mock
from velodyne import VelodyneThread, Velodyne
from johndeere import (JohnDeere, center, go, wait_for_start, 
                       setup_faster_update)
from apyros.metalog import MetaLog, disableAsserts
from apyros.sourcelogger import SourceLogger

from route import Convertor, Route
from line import distance


SAFE_DISTANCE_STOP = 2.5  # meters
SAFE_DISTANCE_GO = SAFE_DISTANCE_STOP + 0.5
TURN_DISTANCE = 4.0
STRAIGHT_EPS = math.radians(10)
NO_TURN_DISTANCE = TURN_DISTANCE + 0.5


def gps_data_extension(robot, id, data):
    if id=='gps':
        robot.gps_data = data
        if robot.localization is not None:
            robot.localization.updateGPS(data) 

def velodyne_data_extension(robot, id, data):
    if id=='velodyne':
        robot.velodyne_data = data

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
    print(gps_log_name)
    if metalog.replay:
        robot.gps = DummySensor()
        function = SourceLogger(None, gps_log_name).get
    else:
        robot.gps = GPS(verbose=0)
        function = SourceLogger(robot.gps.coord, gps_log_name).get
    robot.gps_data = None
    robot.register_data_source('gps', function, gps_data_extension) 

    # Velodyne
    velodyne_log_name = metalog.getLog('velodyne_dist')
    print(velodyne_log_name)
    sensor = Velodyne(metalog=metalog)
    if metalog.replay:
        robot.velodyne = DummySensor()
        function = SourceLogger(None, velodyne_log_name).get
    else:
        robot.velodyne = VelodyneThread(sensor)
        function = SourceLogger(robot.velodyne.scan_safe_dist, velodyne_log_name).get
    robot.velodyne_data = None
    robot.register_data_source('velodyne', function, velodyne_data_extension) 

    robot.gps.start()  # ASAP so we get GPS fix
    robot.velodyne.start()  # the data source is active, so it is necessary to read-out data

    center(robot)
    wait_for_start(robot)

    moving = False
    robot.desired_speed = 0.5
    start_time = robot.time
    prev_gps = robot.gps_data
    prev_destination_dist = None
    while robot.time - start_time < 30*60:  # limit 30 minutes
        robot.update()
        if robot.gps_data != prev_gps:
            print(robot.time, robot.gas, robot.gps_data, robot.velodyne_data)
            prev_gps = robot.gps_data
        dist = None
        if robot.velodyne_data is not None:
            dist_index = None
            if len(robot.velodyne_data) == 2:
                index, dist = robot.velodyne_data
            else:
                index, dist, dist_index = robot.velodyne_data
            if dist is not None:
                dist = min(dist)  # currently supported tupple of readings
        if moving:
            if dist is None or dist < SAFE_DISTANCE_STOP:
                print("!!! STOP !!! -",  robot.velodyne_data)
                #center(robot)
                robot.canproxy.stop()
                moving = False

            elif dist < TURN_DISTANCE:
                if abs(robot.steering_angle) < STRAIGHT_EPS:
                    arr = robot.velodyne_data[1]
                    num = len(arr)
                    left, right = min(arr[:num/2]), min(arr[num/2:])
                    print("DECIDE", left, right, robot.velodyne_data)
                    if left <= right:
                        robot.canproxy.set_turn_raw(-100)
                        robot.steering_angle = math.radians(-30)  # TODO replace by autodetect
                    else:
                        robot.canproxy.set_turn_raw(100)
                        robot.steering_angle = math.radians(30)  # TODO replace by autodetect

            elif dist > NO_TURN_DISTANCE:
                if abs(robot.steering_angle) > STRAIGHT_EPS:
                    robot.canproxy.set_turn_raw(0)
                    robot.steering_angle = 0.0  # TODO replace by autodetect

        else:  # not moving
            if dist is not None and dist > SAFE_DISTANCE_GO:
                print("GO",  robot.velodyne_data)
                #go(robot)
                robot.canproxy.go()
                moving = True
        if not robot.buttonGo:
            print("STOP!")
            break
    robot.canproxy.stop_turn()
    center(robot)
    robot.velodyne.requestStop()
    robot.gps.requestStop()


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
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

