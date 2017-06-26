#!/usr/bin/python
"""
  Navigate given pattern in selected area
  usage:
       ./navpat.py <notes> | <metalog> [<F>] [--view]
"""

import os
import sys
import math
import numpy as np

from apyros.metalog import MetaLog, disableAsserts
from apyros.sourcelogger import SourceLogger

from can import CAN, DummyMemoryLog, ReplayLogInputsOnly, ReplayLog
from johndeere import (JohnDeere, setup_faster_update, ENC_SCALE,
                       emergency_stop_extension, EmergencyStopException)

from driver import go_straight, turn, follow_line_gen
from helper import attach_sensor, detach_all_sensors
from line import Line

from lib.landmarks import ConeLandmarkFinder
from lib.localization import SimpleOdometry


LASER_OFFSET = (1.78, 0.0, 0.39)  # this should be common part?


class NearObstacle:
    pass


def min_dist(data, infinity=None):
    data = np.array(data)
    mask = (data > 0)
    if np.any(mask):
        return np.min(data[mask]) * 0.001
    return infinity

prev_cones = []
prev_near = False
def detect_near_extension(robot, id, data):
    global prev_near
    if id=='laser':
        if data is not None and data != []:
            if prev_near and min_dist(data) < 0.5:
                raise NearObstacle()
            prev_near = min_dist(data) < 1.0
#            prev_near = False # suicide!

            finder = ConeLandmarkFinder()
            global prev_cones
            cones = finder.find_cones(data)
#            print '(%.2f, %.2f, %.3f)' % robot.localization.pose(), finder.match_pairs(prev_cones, cones)
            robot.localization.update_landmarks(id, cones)
            prev_cones = cones
            # TODO:
            #  - collection of all potential cones
            #  - cross distance verification
            #  - "feature tracking"
            #  - localization
            #  - camera verification

viewer_data = []
g_img_dir = None
def viewer_extension(robot, id, data):
    if id == 'laser':
        global viewer_data, g_img_dir
        poses = [robot.localization.pose()]
        x, y, heading = robot.localization.pose()

        scans = [((x, y, 0.0 ), -3.0)]  # hacked color
        laser_pose = x + math.cos(heading)*LASER_OFFSET[0], y + math.sin(heading)*LASER_OFFSET[0], heading
        step = 2
        for i in xrange(0, 540, step):
            dist = data[i]/1000.0
            angle = math.radians(i/2 - 135)
            scans.append((getCombinedPose(laser_pose, (0, 0, angle)), dist))

        image = None
        if robot.camera_data is not None and robot.camera_data[0] is not None:
            assert g_img_dir is not None
            image = os.path.join(g_img_dir, robot.camera_data[0][5:])
        camdir = None
        compass = None

        for raw_angle, raw_dist, raw_width in prev_cones:
            dist = raw_dist/1000.0
            angle = math.radians(raw_angle/2 - 135)
            xx, yy, _ = getCombinedPose(laser_pose, (math.cos(angle)*dist, math.sin(angle)*dist, 0))
            color = (0xFF, 0x80, 0)
            colors = [(0xFF, 0xFF, 0xFF), (0xFF, 0, 0), (0, 0xFF, 0), (0, 0, 0xFF)]
            for cone_xy, cone_color in zip(robot.localization.global_map, colors):
                if math.hypot(xx-cone_xy[0], yy-cone_xy[1]) < 2.0:
                    color = cone_color

            width = raw_width * math.radians(0.5) * raw_dist/1000.0  # in meters
            print "width", width
            if width < 0.05 or width > 0.5:
                color = (128, 128, 128)  # gray
            scans.append( ( (xx, yy, 0), -1.5, color) ) # color param
        record = (poses, scans, image, camdir, compass)
        viewer_data.append(record)
    elif id == 'camera':
        print data


def follow_line(robot, line, speed=None, timeout=None):
    if timeout is None:
        timeout = 20  # TODO set default to 2 * line length * speed

    if speed is not None:
        robot.set_desired_speed(speed)

    start_time = robot.time
    for angle in follow_line_gen(robot, line, stopDistance=0.0, turnScale=4.0, 
                                 offsetSpeed=math.radians(20), offsetDistance=0.03):
        robot.set_desired_steering(angle)
        robot.update()
        if robot.time - start_time > timeout:
            print "TIMEOUT!", timeout
            break


def turn_back(robot, speed):
    turn(robot, math.radians(-60), radius=2.0, speed=speed, with_stop=True, timeout=30.0)  # right
    turn(robot, math.radians(-60), radius=2.0, speed=-speed, with_stop=True, timeout=30.0)  # backup
    turn(robot, math.radians(-60), radius=2.0, speed=speed, with_stop=True, timeout=30.0)  # right again


def run_oval(robot, speed):
    robot.set_desired_speed(speed)
    follow_line(robot, Line((0, 0), (4.0, 0)))
    turn(robot, math.radians(180), radius=2.0, speed=speed, with_stop=False, timeout=20.0)       
    # TODO change second radius once the localization & navigation are repeatable
    follow_line(robot, Line((4.0, 4.0), (0, 4.0)))
    turn(robot, math.radians(180), radius=2.0, speed=speed, with_stop=False, timeout=20.0)


def run_oval(robot, speed):
    robot.set_desired_speed(speed)
    follow_line(robot, Line((5, 0), (10, 0)), speed=speed, timeout=60)
    robot.canproxy.stop()
    turn(robot, math.radians(180), radius=2.5, speed=speed, with_stop=True, timeout=60.0)       
    # TODO change second radius once the localization & navigation are repeatable
    follow_line(robot, Line((10, 5), (5, 5)), speed=speed, timeout=60)
    robot.canproxy.stop()
    turn(robot, math.radians(180), radius=2.5, speed=speed, with_stop=True, timeout=60.0)


def run_there_and_back_SCHOOL(robot, speed):
    follow_line(robot, Line((0, 2.3), (14.0, 2.3)), speed=speed, timeout=60)
    turn_back(robot, speed)
    follow_line(robot, Line((14.0, 2.3), (0, 2.3)), speed=speed, timeout=60)
    turn_back(robot, speed)


def run_there_and_back(robot, speed):
    follow_line(robot, Line((0, 2.5), (15.0, 2.5)), speed=speed, timeout=60)
    turn_back(robot, speed)
    follow_line(robot, Line((15.0, 2.5), (0, 2.5)), speed=speed, timeout=60)
    turn_back(robot, speed)


def navigate_pattern(metalog, viewer=None):
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

#    robot.localization.global_map = [(0.0, 0.0), (13.6, 0.0), (13.6, 4.7), (0.0, 4.7)]  # note not correct!
#    robot.localization.set_pose((0.0, 2.3, 0.0))
    robot.localization.global_map = [(0.0, 0.0), (15.0, 0.0), (15.0, 5.0), (0.0, 5.0)]  # note not correct!
    robot.localization.set_pose((0.0, 2.5, 0.0))
#    robot.localization.set_pose((2.5, 0.0, 0.0))

    robot.canproxy.stop()
    robot.canproxy.set_turn_raw(0)

    if viewer is not None:
        robot.extensions.append(('viewer', viewer_extension))

    speed = 0.5

    try:
        robot.extensions.append(('detect_near', detect_near_extension))
        robot.extensions.append(('emergency_stop', emergency_stop_extension))

        for i in xrange(10):
#            run_oval(robot, speed)
            run_there_and_back(robot, speed)

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
    viewer = None
    if 'meta_' in sys.argv[1]:
        metalog = MetaLog(filename=sys.argv[1])
        if '--view' in sys.argv:
            from tools.viewer import main as viewer_main
            from tools.viewer import getCombinedPose
            viewer = viewer_main
            g_img_dir = os.path.split(sys.argv[1])[0]  # TODO basename??
    elif len(sys.argv) > 2:
        metalog = MetaLog(filename=sys.argv[2])
    if len(sys.argv) > 2 and sys.argv[-1] == 'F':
        disableAsserts()
    
    if metalog is None:
        metalog = MetaLog()

    navigate_pattern(metalog, viewer)
    if viewer is not None:
        viewer(filename=None, posesScanSet=viewer_data)

# vim: expandtab sw=4 ts=4 

