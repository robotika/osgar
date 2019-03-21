"""
    Analyze 3D lidar scan points acquired via slope (approximately 30deg down pointing) sensor
"""
import math

import numpy as np

from osgar.logger import LogReader, lookup_stream_id
from osgar.lib.serialize import deserialize

LIDAR_HEIGHT = 0.625
LIDAR_ANGLE = math.radians(-30) + math.radians(3.5)
LIDAR_Y_SHIFT = 0.0


def analyze(filename, slope_lidar_name, poses_name):
    slope_lidar_id = lookup_stream_id(filename, slope_lidar_name)
    pose_id = lookup_stream_id(filename, poses_name)
#    print('x,y,z')
    pose2d = 0, 0, 0
    for dt, channel, data in LogReader(filename, only_stream_id=[slope_lidar_id, pose_id]):
        if channel == pose_id:
            pose2d = deserialize(data)
#            print('pose2d', dt, pose2d)
            continue
        scan = deserialize(data)
#        print(dt, len(scan))
        diff = [abs(prev - curr) for prev, curr in zip(scan[:-1], scan[1:])]
#        print(diff[270:-270])
#        print(diff[135:-135])
#        print(diff[400:-400])
        pts = []
        for i, dist_mm in enumerate(scan):
            angle = i * math.radians(270)/len(scan) - math.radians(135)
            if abs(angle) > math.radians(90):
                continue
            dist = dist_mm / 1000.0

            x = dist * math.cos(LIDAR_ANGLE) * math.cos(angle) + LIDAR_Y_SHIFT
            y = dist * math.sin(angle)
            z = LIDAR_HEIGHT + dist * math.sin(LIDAR_ANGLE) * math.cos(angle)
            px, py, pheading = pose2d
            x += px/1000.0
            pts.append((x, y, z))
#            print('%f,%f,%f' % (x, y, z))
#            print('%.3f %.3f %.3f' % (x, y, z))
            print('%.3f\t%.3f\t%.3f' % (x, y, z))

        z_arr = np.array([z for x, y, z in pts])
        mask = abs(z_arr) < 0.1
#        print(dt, np.argmax(mask), np.argmax(np.flip(mask, axis=0))) #, scan[400:405])
#        print([int(z*1000) for x, y, z in pts[135:-135]])


if __name__ == '__main__':
    import argparse

    arg_parser = argparse.ArgumentParser(description='Analyze slope lidar',
                                         formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    arg_parser.add_argument('logfile', help='Path to logfile')
    arg_parser.add_argument('--lidar', help='slope lidar stream name', default='slope_lidar.scan')
    arg_parser.add_argument('--poses', help='poses stream name', default='eduro.pose2d')

    args = arg_parser.parse_args()

    analyze(args.logfile, args.lidar, args.poses)

# vim: expandtab sw=4 ts=4
