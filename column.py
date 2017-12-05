#!/usr/bin/python
"""
  Analysis of Velodyne VLP-16 data - column detection
  usage:
       ./column.py <metalog>
"""
import sys
import math
from itertools import combinations
from apyros.metalog import MetaLog
from velodyne import Velodyne

MAX_TOLERANCE = 0.1  # fraction (max-min)/min
MAX_COLUMN_WIDTH = 0.12  # in meters
EXPECTED_COLUMN_DISTANCE = 3.0  # meters

def print_pose(pose):
    if pose is not None:
        print("%.2f, %.2f, %.1f" % (pose[0], pose[1], math.degrees(pose[2])))

def print_scan(scan):
    for i, arr in enumerate(scan):
        print("%d:" % i, arr)


def polar2coord( polar_coord ):
    "convert velodyne polar coordinates (deg, dist) into cartesian coord"
    deg, dist = polar_coord
    angle = math.radians(360 - deg)  # Velodyne has clockwise rotation
    return dist * math.cos(angle), dist * math.sin(angle)

def col_dist( lower, upper ):
    "compute distance of two columns in polar coordinates"
    a = polar2coord(lower)
    b = polar2coord(upper)
    return math.hypot(a[0]-b[0], a[1]-b[1])

def analyse_pose(prev_pose, new_data):
    best = None
    for lower, upper in combinations(new_data, 2):
        # or just use combinations?
        dist = col_dist(lower, upper)
#        print dist
        if best is None or \
                abs(best[0] - EXPECTED_COLUMN_DISTANCE) > abs(dist - EXPECTED_COLUMN_DISTANCE):            
            best = dist, lower, upper
    if best is None:
        # later maybe extrapolation from old+new partial data?
        return None
#    print "----------------"

    dist, lower, upper = best
    a = polar2coord(lower)
    b = polar2coord(upper)
    angle = math.radians(90) - math.atan2(b[1] - a[1], b[0] - a[0])
    t = -(a[0]+b[0])/2., -(a[1]+b[1])/2.
    ca, sa = math.cos(angle), math.sin(angle)
    x, y = t[0]*ca - t[1]*sa, t[0]*sa + t[1]*ca  # TODO check
    return (x, y, angle), dist


def detect_columns(scan):
    data = []
    for arr in scan:
        mask = arr > 0
        mask[0] = False   # -15deg scan sees often ground
        if max(mask):
            data.append(arr[mask].min())
        else:
            data.append(10000)  # None??

    MIN_STEP = 500  # 1m
    MAX_VAR = 100 # 20cm, probably too much

    assert len(data) == 360, len(data)
    ret = []
    for i in range(len(data)):
        if data[i-1] - MIN_STEP <= data[i]:
            continue
        tmp = []
        while data[i-1] + MIN_STEP >= data[i]:
            tmp.append(data[i])
            if abs(int(data[i]) - tmp[0]) > MAX_VAR:
                tmp = []
                break
            i += 1
            if i >= len(data):
                break
        if len(tmp) > 0:
            print(i, tmp)
            ret.append( (i + len(tmp)/2, 0.002*sum(tmp)/len(tmp)) )
    return ret


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(2)
    assert 'meta_' in sys.argv[1], sys.argv[1]
    metalog = MetaLog(filename=sys.argv[1])
    sensor = Velodyne(metalog=metalog)


    prev = None
    for ii in range(20000):
        sensor.update()
        curr = sensor.scan_index, sensor.safe_dist
        if prev != curr:
            if sensor.scan_index % 1 == 0:
                print('-----', sensor.scan_index, '-----')
                data = detect_columns(sensor.dist)
                pose = analyse_pose(None, data)
                if pose is not None:
                    pose, dist = pose
                    print_pose(pose)
                    print(dist)

            prev = curr

# vim: expandtab sw=4 ts=4 

