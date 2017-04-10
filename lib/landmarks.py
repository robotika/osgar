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

# vim: expandtab sw=4 ts=4
