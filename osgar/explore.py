"""
  Explore - follow a wall/obstacles
    (use 2D SICK LIDAR only)
"""
import math
from datetime import timedelta

import numpy as np

from osgar.node import Node


SAFE_DISTANCE_STOP = 0.5
SAFE_DISTANCE_GO = SAFE_DISTANCE_STOP + 0.3
WALL_DISTANCE = 1.0 # m
DESIRED_SPEED = 0.5  # m/s


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
    return min_dist(data[num//2:]), min_dist(data[:num//2])


def tangent_circle(dist, radius):
    if dist < 2 * radius:
        if dist >= radius:
            return math.asin(radius/float(dist))
        return math.radians(100)
    return None


def follow_wall_angle(laser_data, radius):
    data = np.array(laser_data)
    size = len(laser_data)
    mask = (data <= 10)  # ignore internal reflections
    data[mask] = 20000
    index = np.argmin(data[:size//2])  # only right side
    dist = data[index]/1000.0
    laser_angle = math.radians((-270+index)/3.0)  # TODO angular resolution
    angle = tangent_circle(dist, radius)
    if angle is not None:
        # print '(%d, %.3f) %.1f' % (index, dist, math.degrees(laser_angle + angle))
        return laser_angle + angle
    return None


class FollowWall(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)

    def send_speed_cmd(self, speed, angular_speed):
        return self.publish('desired_speed', [round(speed*1000), round(math.degrees(angular_speed)*100)])

    def update(self):  # hack, this method should be called run instead!       
        channel = super().update()  # define self.time

        moving = False
        desired_speed = 0.0
        start_time = self.time
        while self.time - start_time < timedelta(minutes=1):
            channel = channel = super().update()
            if channel == 'scan':
                assert len(self.scan) == 811, len(self.scan)  # 541
                distL, distR = min_dist_arr(self.scan[270:-270])  # was 200
                distL = 20.0 if distL is None else distL
                distR = 20.0 if distR is None else distR
                dist = min(distL, distR)
                desired_angular_speed = follow_wall_angle(self.scan, radius=WALL_DISTANCE)
                print(desired_angular_speed)
                if moving:
                    if dist is None or dist < SAFE_DISTANCE_STOP:
                        print("!!! STOP !!!",  dist, (distL, distR))
                        desired_speed = 0.0
                        desired_angular_speed = 0.0
                        moving = False
                else:  # not moving
                    if dist > SAFE_DISTANCE_GO:
                        print("GO",  dist)
                        desired_speed = DESIRED_SPEED
                        moving = True
                self.send_speed_cmd(desired_speed, desired_angular_speed)

            elif channel == 'emergency_stop':
                self.send_speed_cmd(0.0, 0.0)
                self.request_stop()  # it should be "delayed"


if __name__ == "__main__":
    from osgar.logger import LogWriter, LogReader
    import argparse
    import sys

    from osgar.lib.config import load as config_load
    from osgar.record import Recorder

    parser = argparse.ArgumentParser(description='Follow Wall')
    subparsers = parser.add_subparsers(help='sub-command help', dest='command')
    subparsers.required = True
    parser_run = subparsers.add_parser('run', help='run on real HW')
    parser_run.add_argument('config', nargs='+', help='configuration file')
    parser_run.add_argument('--note', help='add description')

    parser_replay = subparsers.add_parser('replay', help='replay from logfile')
    parser_replay.add_argument('logfile', help='recorded log file')
    parser_replay.add_argument('--force', '-F', dest='force', action='store_true', help='force replay even for failing output asserts')
    parser_replay.add_argument('--config', nargs='+', help='force alternative configuration file')
    args = parser.parse_args()

    if args.command == 'replay':
        from osgar.replay import replay
        args.module = 'app'
        game = replay(args, application=FollowWall)
        game.run()

    elif args.command == 'run':
        log = LogWriter(prefix='wall-', note=str(sys.argv))
        config = config_load(*args.config)
        log.write(0, bytes(str(config), 'ascii'))  # write configuration
        recorder = Recorder(config=config['robot'], logger=log, application=FollowWall)
        game = recorder.modules['app']  # TODO nicer reference
        recorder.start()
        try:
            game.join()
        except KeyboardInterrupt:
            print('USER shut down!')
        recorder.finish()

# vim: expandtab sw=4 ts=4 

