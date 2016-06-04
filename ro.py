#!/usr/bin/python
"""
  RoboOrienteering contest with John Deere
  usage:
       ./ro.py <task> [<metalog> [<F>]]
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

LEFT_TURN_TIME = 1.2
RIGHT_TURN_TIME = 1.5

def gps_data_extension(robot, id, data):
    if id=='gps':
        robot.gps_data = data
        if robot.localization is not None:
            robot.localization.updateGPS(data) 

def velodyne_data_extension(robot, id, data):
    if id=='velodyne':
        robot.velodyne_data = data

def test_gas( robot ):
    for ii in range(10):
        robot.pulse_forward( 0.3 )
        robot.wait(10)
        robot.pulse_backward( 0.3 )
        robot.wait(10)


def test_turn( robot ):
    for ii in range(3):
        robot.pulse_left(0.8)  # should be in ratio 4:5
        robot.wait(2)
        robot.pulse_right(1.0)
        robot.wait(2)


def test_drop_ball(robot):
    print "TEST BALLS"
    for ii in xrange(3):
        print "DROP BALL"
        robot.drop_ball = True
        robot.wait(1.0)
        print "CLOSE"
        robot.drop_ball = False
        robot.wait(3.0)

def ver0(metalog, waypoints=None):
    assert metalog is not None
    assert waypoints is not None  # for simplicity (first is start)

    conv = Convertor(refPoint = waypoints[0]) 
    waypoints = waypoints[1:-1]  # remove start/finish

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


    # mount_sensor(GPS, robot, metalog)
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

    # mount_sensor(VelodyneThread, robot, metalog)
    velodyne_log_name = metalog.getLog('velodyne_dist')
    print velodyne_log_name
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
    while robot.time - start_time < 30*60:  # RO timelimit 30 minutes
        robot.update()
        if robot.gps_data != prev_gps:
            print robot.time, robot.gas, robot.gps_data, robot.velodyne_data
            prev_gps = robot.gps_data
            if robot.gps_data is not None:
                dist_arr = [distance( conv.geo2planar((robot.gps_data[1], robot.gps_data[0])), 
                                      conv.geo2planar(destination) ) for destination in waypoints]
                dist = min(dist_arr)
                print "DIST-GPS", dist
                if prev_destination_dist is not None:
                    if prev_destination_dist < dist and dist < 10.0:
                        robot.drop_ball = True
                        # remove nearest
                        i = dist_arr.index(dist)  # ugly, but ...
                        print "INDEX", i
                        del waypoints[i]
                prev_destination_dist = dist

        dist = None
        if robot.velodyne_data is not None:
            index, dist = robot.velodyne_data
            if dist is not None:
                dist = min(dist)  # currently supported tupple of readings
        if moving:
            if dist is None or dist < SAFE_DISTANCE_STOP:
                print "!!! STOP !!! -",  robot.velodyne_data
                center(robot)
                moving = False

            elif dist < TURN_DISTANCE:
                if abs(robot.steering_angle) < STRAIGHT_EPS:
                    arr = robot.velodyne_data[1]
                    num = len(arr)
                    left, right = min(arr[:num/2]), min(arr[num/2:])
                    print "DECIDE", left, right, robot.velodyne_data
                    if left <= right:
                        robot.pulse_right(RIGHT_TURN_TIME)
                        robot.steering_angle = math.radians(-30)  # TODO replace by autodetect
                    else:
                        robot.pulse_left(LEFT_TURN_TIME)
                        robot.steering_angle = math.radians(30)  # TODO replace by autodetect

            elif dist > NO_TURN_DISTANCE:
                if abs(robot.steering_angle) > STRAIGHT_EPS:
                    if robot.steering_angle < 0:
                        robot.pulse_left(LEFT_TURN_TIME)
                    else:
                        robot.pulse_right(RIGHT_TURN_TIME)
                    robot.steering_angle = 0.0  # TODO replace by autodetect

        else:  # not moving
            if dist is not None and dist > SAFE_DISTANCE_GO:
                print "GO",  robot.velodyne_data
                go(robot)
                moving = True
        if not robot.buttonGo:
            print "STOP!"
            break
    center(robot)
    robot.velodyne.requestStop()
    robot.gps.requestStop()


def load_waypoints(filename):
    arr = []
    for line in open(filename):
        s = line.split()
        if len(s) >= 3:
            arr.append( (float(s[2]), float(s[1])) )  # lon, lat format
    return arr

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

    waypoints = None
    if sys.argv[1].endswith('.dat'):
        waypoints = load_waypoints(sys.argv[1])
    ver0(metalog, waypoints)


# There are too many TODOs at the moment, including basic CAN modules integration (EmergencySTOP etc)
# Common launcher will be postponed.
#if __name__ == "__main__":
#    from johndeere import JohnDeere
#    import launcher
#    launcher.launch(sys.argv, JohnDeere, RoboOrienteering, configFn=setup_faster_update) 

# vim: expandtab sw=4 ts=4 

