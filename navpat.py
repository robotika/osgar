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
import itertools

from johndeere import emergency_stop_extension, EmergencyStopException

from driver import go_straight, turn, follow_line_gen
from helper import attach_processor
from line import Line
from launcher import parse_and_launch, LASER_OFFSET, viewer_scans_append, getCombinedPose

from lib.landmarks import ConeLandmarkFinder
from lib.camera_marks import find_cones


class NearObstacle(Exception):
    pass

class NoLaserData(Exception):
    pass


def min_dist(data, infinity=None):
    data = np.array(data)
    mask = (data > 0)
    if np.any(mask):
        return np.min(data[mask]) * 0.001
    return infinity


class LaserDetector:
    def __init__(self):
        self.prev_cones = []
        self.prev_near = False
        self.last_laser_update_time = None

    def near_range_extension(self, robot, id, data):
        if self.last_laser_update_time is None:
            # well, we do not want to stop the machine immediately - there could be
            # first update of some other sensor
            # TODO review no data from the beginning
            self.last_laser_update_time = robot.time

        if id=='laser':
            self.last_laser_update_time = robot.time
            if data is not None and data != []:
                if self.prev_near and min_dist(data) < 0.5:
                    raise NearObstacle()
                self.prev_near = min_dist(data) < 1.0
#                self.prev_near = False # suicide!

                finder = ConeLandmarkFinder()
                cones = finder.find_cones(data)
#                print '(%.2f, %.2f, %.3f)' % robot.localization.pose(), finder.match_pairs(prev_cones, cones)
                robot.localization.update_landmarks(id, cones)
                self.prev_cones = cones
                # TODO:
                #  - collection of all potential cones
                #  - cross distance verification
                #  - "feature tracking"
                #  - localization
                #  - camera verification

        elif self.last_laser_update_time + 1.0 < robot.time:
            raise NoLaserData()


    def viewer_extension(self, robot, id, data):
        if id == 'laser':
            x, y, heading = robot.localization.pose()
            laser_pose = x + math.cos(heading)*LASER_OFFSET[0], y + math.sin(heading)*LASER_OFFSET[0], heading

            selected = []
            for raw_angle, raw_dist, raw_width in self.prev_cones:
                dist = raw_dist/1000.0
                angle = math.radians(raw_angle/2 - 135)
                xx, yy, _ = getCombinedPose(laser_pose, (math.cos(angle)*dist, math.sin(angle)*dist, 0))
                color = (0xFF, 0x80, 0)
                colors = [(0xFF, 0xFF, 0xFF), (0xFF, 0, 0), (0, 0xFF, 0), (0, 0, 0xFF)]
                for i, cone in enumerate(zip(robot.localization.global_map, colors)):
                    cone_xy, cone_color = cone
                    if math.hypot(xx-cone_xy[0], yy-cone_xy[1]) < 2.0:
                        color = cone_color
                        selected.append( (i, (raw_angle, raw_dist)) )

                width = raw_width * math.radians(0.5) * raw_dist/1000.0  # in meters
                print("width", width)
                if width < 0.05 or width > 0.5:
                    color = (128, 128, 128)  # gray
                viewer_scans_append( ( (xx, yy, 0), -1.5, color) ) # color param

            if len(selected) >= 2:
                finder = ConeLandmarkFinder()
                for a, b in itertools.combinations(sorted(selected), 2):
                    print("selected\t%f\t%d\t%d\t%f\n" % (robot.time, a[0], b[0], finder.pair_distance(a[1], b[1])))


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
            print("TIMEOUT!", timeout)
            break


def turn_back(robot, speed):
    turn(robot, math.radians(-60), radius=2.0, speed=speed, with_stop=True, timeout=30.0)  # right
    turn(robot, math.radians(-60), radius=2.0, speed=-speed, with_stop=True, timeout=30.0)  # backup
    turn(robot, math.radians(-60), radius=2.0, speed=speed, with_stop=True, timeout=30.0)  # right again


def nav2cone(robot, speed, laser_detector):
    """navigate to cone, which should be in front of you"""
    cone_dist = None
    while cone_dist is None or cone_dist > 2.0:  # TODO add also timeout 
        robot.update()
        cones = laser_detector.prev_cones  # TODO use robot.prev_cones instead of global variable
        print("Cones:", cones)
        if len(cones) == 0:
            # well, shold we move?? maybe based on history?
            robot.stop()
        else:
            # find cone in front of the robot
            cone = cones[0]
            center_raw = 270  # 270deg with 0.5deg resolution
            for c in cones[1:]:
                if abs(c[0] - center_raw) < abs(cone[0] - center_raw):
                    cone = c
            cone_dist = cone[1]/1000.0

            acceptable_angle_raw = 45*2  # 45deg with 0.5 resolution
            if abs(cone[0] - center_raw) < acceptable_angle_raw:
                robot.set_desired_steering( math.radians(0.5*(cone[0] - center_raw)) )
                robot.set_desired_speed(speed)
            else:
                robot.stop()


def run_mapping(robot, speed, laser_detector):
    for i in range(4):
        nav2cone(robot, speed, laser_detector)
        turn(robot, math.radians(75), radius=2.0+1.5, speed=speed, with_stop=False, timeout=60.0)


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


def run_fill_pattern(robot, long_side, speed, conf):
    STEP = conf['step']
    RAD1 = 2.0
    RAD2 = RAD1 + STEP/2.0
    SAFETY = 2.0
    for i in range(10):
        Y = i * STEP
        follow_line(robot, Line((RAD1+SAFETY, Y), (long_side-RAD2-SAFETY, Y)), speed=speed, timeout=60)
        turn(robot, math.radians(180), radius=RAD2, speed=speed, with_stop=True, timeout=60.0)
        follow_line(robot, Line((long_side-RAD2-SAFETY, Y+2*RAD2), (RAD1+SAFETY, Y+2*RAD2)), speed=speed, timeout=60)
        turn(robot, math.radians(180), radius=RAD1, speed=speed, with_stop=True, timeout=60.0)


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

    if len(robot.localization.global_map) > 0:
        long_side = max([x for x, y in robot.localization.global_map])
    else:
        long_size = None

    robot.canproxy.stop()
    robot.canproxy.set_turn_raw(0)

    laser_detector = LaserDetector()    

    if viewer is not None:
        robot.extensions.append(('navpat_viewer', laser_detector.viewer_extension))

    speed = 0.5

    try:
        robot.extensions.append(('detect_near', laser_detector.near_range_extension))

        for i in range(10):
#            run_oval(robot, speed)
            if conf is None:
                run_there_and_back(robot, long_side, speed)
            elif conf['pattern'] == 'fill':
                run_fill_pattern(robot, long_side, speed, conf)
            elif conf['pattern'] == 'mapping':
                run_mapping(robot, speed, laser_detector)
            else:
                assert False, conf['pattern']  # unknown pattern

    except NearObstacle:
        print("Near Exception Raised!")
        robot.extensions = []  # hack
    except NoLaserData:
        print("!!!ERROR!!! Missing laser updates for last {:.1f}s".format(robot.time - laser_detector.last_laser_update_time))
        robot.extensions = []  # hack

    robot.canproxy.stop()
    robot.canproxy.stop_turn()
    robot.wait(3.0)
    


if __name__ == "__main__":
    with parse_and_launch() as (robot, metalog, config, viewer):
        config = config.data.get('navpat')  # TODO move this to launcher
        navigate_pattern(robot, metalog, config, viewer)

# vim: expandtab sw=4 ts=4 

