#!/usr/bin/python
"""
  Navigate given pattern in selected area
  usage:
       ./navpat.py [-h] {run,replay} ...

positional arguments:
  {run,replay}  sub-command help
    run         run on real HW
    replay      replay from logfile

optional arguments:
  -h, --help       show this help message and exit
"""

import math
import numpy as np
import cv2

from johndeere import emergency_stop_extension, EmergencyStopException

from driver import go_straight, turn, follow_line_gen
from helper import attach_processor
from line import Line
from launcher import parse_and_launch, LASER_OFFSET, viewer_scans_append, getCombinedPose

from lib.landmarks import ConeLandmarkFinder
from lib.camera_marks import find_cones


class NearObstacle:
    pass

class NoLaserData:
    pass


def min_dist(data, infinity=None):
    data = np.array(data)
    mask = (data > 0)
    if np.any(mask):
        return np.min(data[mask]) * 0.001
    return infinity

prev_cones = []
prev_near = False
last_laser_update_time = None

def detect_near_extension(robot, id, data):
    global prev_near, last_laser_update_time
    if last_laser_update_time is None:
        # well, we do not want to stop the machine immediately - there could be
        # first update of some other sensor
        # TODO review no data from the beginning
        last_laser_update_time = robot.time

    if id=='laser':
        last_laser_update_time = robot.time
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
    
    elif last_laser_update_time + 1.0 < robot.time:
        raise NoLaserData()


def navpat_viewer_extension(robot, id, data):
    if id == 'laser':
        x, y, heading = robot.localization.pose()
        laser_pose = x + math.cos(heading)*LASER_OFFSET[0], y + math.sin(heading)*LASER_OFFSET[0], heading

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
            viewer_scans_append( ( (xx, yy, 0), -1.5, color) ) # color param


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


def run_there_and_back(robot, long_side, speed):
    follow_line(robot, Line((0, 2.5), (long_side, 2.5)), speed=speed, timeout=60)
    turn_back(robot, speed)
    follow_line(robot, Line((long_side, 2.5), (0, 2.5)), speed=speed, timeout=60)
    turn_back(robot, speed)


def image_callback(data):
    assert len(data) > 1
    filename = data[0]
    img = cv2.imread(filename)
    if img is not None:
        cones= find_cones(img)
        return (data, cones)
    return (data, None)


def navigate_pattern(robot, metalog, conf, viewer=None):
    attach_processor(robot, metalog, image_callback)

    long_side = max([x for x, y in robot.localization.global_map])

    robot.canproxy.stop()
    robot.canproxy.set_turn_raw(0)

    if viewer is not None:
        robot.extensions.append(('navpat_viewer', navpat_viewer_extension))

    speed = 0.5

    try:
        robot.extensions.append(('detect_near', detect_near_extension))

        for i in xrange(10):
#            run_oval(robot, speed)
            run_there_and_back(robot, long_side, speed)

    except NearObstacle:
        print "Near Exception Raised!"
        robot.extensions = []  # hack
    except NoLaserData:
        print "!!!ERROR!!! Missing laser updates for last {:.1f}s".format(robot.time - last_laser_update_time)
        robot.extensions = []  # hack

    robot.canproxy.stop()
    robot.canproxy.stop_turn()
    robot.wait(3.0)
    


if __name__ == "__main__":
    with parse_and_launch() as (robot, metalog, config, viewer):
        navigate_pattern(robot, metalog, config, viewer)

# vim: expandtab sw=4 ts=4 

