"""
  Laser landmarks detector
"""

import numpy as np

MAX_CONE_SIZE = 0.3
ZONE_RADIUS = 2.0  # expected empty space behind cone


def min_dist(data):
    data = np.array(data)
    mask = (data > 0)
    if np.any(mask):
        return np.min(data[mask]) * 0.001
    return None


class ConeLandmarkFinder(object):

    def __init__(self, verbose=False):
        self.verbose = verbose

    def find_cones(self, raw_laser_data):
        step = 10
        data = np.array(raw_laser_data)
        mask = (data == 0)
        data[mask] = 65535  # workaround to simplify arg_min

        arr = []
        for i in xrange(0, len(data), step):
            arr.append(min_dist(data[i:i+step]))

        ret = []
        for i in xrange(1, len(arr) - 3):
            if (arr[i] is not None and
               (arr[i-1] is None or arr[i] < arr[i-1] - ZONE_RADIUS) and
               (arr[i+1] is None or arr[i] < arr[i+1] - ZONE_RADIUS)):
                ii = data[i*step:(i+1)*step].argmin() + i*step
                ret.append( (ii, data[ii]) )
            elif (arr[i] is not None and arr[i+1] is not None and
                 (abs(arr[i] - arr[i+1]) < MAX_CONE_SIZE) and
                 (arr[i-1] is None or arr[i] < arr[i-1] - ZONE_RADIUS) and
                 (arr[i+2] is None or arr[i] < arr[i+2] - ZONE_RADIUS)):
                ii = data[i*step:(i+2)*step].argmin() + i*step
                ret.append( (ii, data[ii]) )
        return ret

    def match_pairs(self, old, new):
        if len(old)==0 or len(new)==0:
            return []
        s = [((angle, dist), 0) for angle, dist in old]
        s += [((angle, dist), 1) for angle, dist in new]
        s = sorted(s)
        ret = []
        for prev, curr in zip(s[:-1], s[1:]):
            if abs(prev[0][0]-curr[0][0]) > 2*5:
                # accept only angle difference smaller than 5deg
                continue
            if abs(prev[0][1]-curr[0][1]) > 200:
                # ignore distances difference bigger than 20cm
                continue
            if prev[1] < curr[1]:
                ret.append((prev[0], curr[0]))
            elif prev[1] > curr[1]:
                ret.append((curr[0], prev[0]))
        return ret

# vim: expandtab sw=4 ts=4
