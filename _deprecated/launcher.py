"""
  Command line parser and context manager for launching John Deere robot
"""

import argparse
import os
import sys
import math
import zipfile
from contextlib import contextmanager

from apyros.metalog import MetaLog, disableAsserts, isMetaLogName
from apyros.sourcelogger import SourceLogger

from can import CAN, DummyMemoryLog, ReplayLogInputsOnly, ReplayLog
from johndeere import (JohnDeere, setup_faster_update, ENC_SCALE,
                       emergency_stop_extension, EmergencyStopException)
from helper import attach_sensor, detach_all_sensors

from lib.localization import SimpleOdometry
from lib.config import Config


LASER_OFFSET = (1.78, 0.0, 0.39)  # this should be common part?
DEFAULT_ZIP_CONFIG = 'config.json'  # default name stored in newer zipped log files


def getCombinedPose( pose, sensorPose ):
  x = pose[0] + sensorPose[0] * math.cos( pose[2] ) - sensorPose[1] * math.sin( pose[2] )
  y = pose[1] + sensorPose[0] * math.sin( pose[2] ) + sensorPose[1] * math.cos( pose[2] )
  heading = sensorPose[2] + pose[2]
  return (x, y, heading)


viewer_data = []
g_img_dir = None
def launcher_viewer_extension(robot, id, data):
    if id == 'laser':
        global viewer_data, g_img_dir
        poses = [robot.localization.pose()]
        x, y, heading = robot.localization.pose()

        scans = [((x, y, 0.0 ), -3.0)]  # hacked color
        laser_pose = x + math.cos(heading)*LASER_OFFSET[0], y + math.sin(heading)*LASER_OFFSET[0], heading
        step = 2
        for i in range(0, 540, step):
            dist = data[i]/1000.0
            angle = math.radians(i/2 - 135)
            scans.append((getCombinedPose(laser_pose, (0, 0, angle)), dist))

        image = None
        if robot.camera_data is not None and robot.camera_data[0] is not None:
            assert g_img_dir is not None
            image = os.path.join(g_img_dir, robot.camera_data[0][5:])
        camdir = None
        compass = None
        record = (poses, scans, image, camdir, compass)
        viewer_data.append(record)
    elif id == 'camera':
        print(data)


def viewer_scans_append(beacon_record):
    """add beacon info (for example "cones") into last viewer record"""
    # TODO not nice because it expects that "launcher_viewer_extension" was called before (which is true)
    global viewer_data
    assert len(viewer_data) > 0
    # (poses, scans, image, camdir, compass)
    viewer_data[-1][1].append(beacon_record)


def create_robot(metalog, conf):
    assert metalog is not None
    assert conf is not None  # config is required!

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

    loc = SimpleOdometry.from_dict(conf.data.get('localization'))
    robot = JohnDeere(can=can, localization=loc, config=conf.data.get('johndeere'))
    robot.UPDATE_TIME_FREQUENCY = 20.0  # TODO change internal and integrate setup

    return robot


@contextmanager
def parse_and_launch():
    """parse sys.argv arguments and return initialized robot"""

    parser = argparse.ArgumentParser(description='Navigate given pattern in selected area')
    subparsers = parser.add_subparsers(help='sub-command help', dest='command')
    subparsers.required = True
    parser_run = subparsers.add_parser('run', help='run on real HW')
    parser_run.add_argument('config', help='configuration file')
    parser_run.add_argument('--note', help='add description')

    parser_replay = subparsers.add_parser('replay', help='replay from logfile')
    parser_replay.add_argument('logfile', help='recorded log file')
    parser_replay.add_argument('--view', dest='view', action='store_true', help='view parsed log')
    parser_replay.add_argument('--force', '-F', dest='force', action='store_true', help='force replay even for failing output asserts')
    parser_replay.add_argument('--config', dest='config', help='use different configuration file')
    args = parser.parse_args()
    conf = None
    if args.config is not None:
        conf = Config.load(args.config)

    viewer = None
    if args.command == 'replay':
        metalog = MetaLog(args.logfile)
        if conf is None and args.logfile.endswith('.zip'):
            if DEFAULT_ZIP_CONFIG in zipfile.ZipFile(args.logfile).namelist():
                s = str(zipfile.ZipFile(args.logfile).read(DEFAULT_ZIP_CONFIG), 'utf-8')
                conf = Config.loads(s)
        if args.view:
            global g_img_dir
            from tools.viewer import main as viewer_main
            from tools.viewer import getCombinedPose
            viewer = viewer_main
            if args.logfile.endswith('.zip'):
                g_img_dir = args.logfile
            else:
                g_img_dir = os.path.dirname(args.logfile)
        if args.force:
            disableAsserts()

    elif args.command == 'run':
        metalog = MetaLog()

    else:
        assert False, args.command   # unsupported command

    robot = create_robot(metalog, conf)

    for sensor_name in ['gps', 'laser', 'camera']:
        attach_sensor(robot, sensor_name, metalog)

    if viewer is not None:
        robot.extensions.append(('launcher_viewer', launcher_viewer_extension))

    try:
        robot.extensions.append(('emergency_stop', emergency_stop_extension))
        yield robot, metalog, conf, viewer
    except EmergencyStopException:
        print("Emergency STOP Exception!")
        robot.extensions = []  # hack

    # TODO make this code conditional based on the state of finished application
    robot.canproxy.stop()
    robot.canproxy.stop_turn()
    robot.wait(3.0)

    detach_all_sensors(robot)

    if viewer is not None:
        viewer(filename=None, posesScanSet=viewer_data)


# vim: expandtab sw=4 ts=4 

