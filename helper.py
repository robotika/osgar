#!/usr/bin/python
"""
  Helper functions for attaching various sensors
"""
from gps import GPS
from gps import DummyGPS as DummySensor  # TODO move to apyros, as mock
from laser import LaserIP
from camera import Camera
from lib.processor import Processor
from apyros.metalog import MetaLog, disableAsserts
from apyros.sourcelogger import SourceLogger

def gps_data_extension(robot, id, data):
    if id=='gps':
        robot.gps_data = data
        if robot.localization is not None:
            robot.localization.updateGPS(data) 


def laser_data_extension(robot, id, data):
    if id=='laser':
        robot.laser_data = data


def remission_data_extension(robot, id, data):
    if id=='remission':
        robot.remission_data = data


def camera_data_extension(robot, id, data):
    if id=='camera':
        robot.camera_data = data


def proc_data_extension(robot, id, data):
    if id=='proc':
        robot.proc_data = data
        print "Received", robot.proc_data


def attach_sensor(robot, sensor_name, metalog):
    assert sensor_name in ['gps', 'laser', 'camera'], sensor_name

    if sensor_name == 'gps':
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
        robot.gps.start()  # ASAP so we get GPS fix
        robot.threads.append(robot.gps)

    elif sensor_name == 'laser':
        # Laser
        laser_log_name = metalog.getLog('laser')
        remission_log_name = metalog.getLog('remission')
        print laser_log_name, remission_log_name
        if metalog.replay:
            robot.laser = DummySensor()
            function = SourceLogger(None, laser_log_name).get
            if remission_log_name is not None:
                # remission is only optional, missing in some old log files
                function2 = SourceLogger(None, remission_log_name).get
        else:
            robot.laser = LaserIP(remission=True)
            robot.laser.stopOnExit = False  # for faster boot-up
            function = SourceLogger(robot.laser.scan, laser_log_name).get
            function2 = SourceLogger(robot.laser.remission, remission_log_name).get
        robot.laser_data = None
        robot.register_data_source('laser', function, laser_data_extension)
        robot.remission_data = None
        if  remission_log_name is not None:
            robot.register_data_source('remission', function2, remission_data_extension)
        robot.laser.start()
        robot.threads.append(robot.laser)

    elif sensor_name == 'camera':
        # Camera
        camera_log_name = metalog.getLog('camera')
        print camera_log_name
        if metalog.replay:
            robot.camera = DummySensor()
            function = SourceLogger(None, camera_log_name).get
        else:
            robot.camera = Camera(sleep=0.2)  # TODO
            function = SourceLogger(robot.camera.lastResult, camera_log_name).get
        robot.camera_data = None
        robot.register_data_source('camera', function, camera_data_extension)
        robot.camera.start()
        robot.threads.append(robot.camera)

    else:
        assert False, sensor_name  # unsuported sensor


def camera_filler(robot, id, data):
    if id=='camera':
        robot.proc.push_back(data)


def attach_processor(robot, metalog, callback):
    # TODO assert called at most once
    name = 'proc'
    robot.proc_data = None
    proc_log_name = metalog.getLog(name)
    if proc_log_name is None:
        print "Processor is not logged"

        return False
    print proc_log_name
    if metalog.replay:
        robot.proc = DummySensor()
        function = SourceLogger(None, proc_log_name).get
    else:
        robot.extensions.append(('proc_in', camera_filler)) 
        robot.proc = Processor(callback)
        function = SourceLogger(robot.proc.get_result, proc_log_name).get
    robot.register_data_source('proc', function, proc_data_extension) 
    robot.proc.start()
    robot.threads.append(robot.proc)
    return True


def detach_all_sensors(robot):
    # TODO unregister all modules
    for thr in robot.threads:
        thr.requestStop()

# vim: expandtab sw=4 ts=4 

