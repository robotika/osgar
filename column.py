#!/usr/bin/python
"""
  Analysis of Velodyne VLP-16 data - column detection
  usage:
       ./column.py <metalog>
"""
import sys
import math
from apyros.metalog import MetaLog
from velodyne import Velodyne

MAX_TOLERANCE = 0.1  # fraction (max-min)/min
MAX_COLUMN_WIDTH = 0.12  # in meters

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print __doc__
        sys.exit(2)
    assert 'meta_' in sys.argv[1], sys.argv[1]
    metalog = MetaLog(filename=sys.argv[1])
    sensor = Velodyne(metalog=metalog)


    prev = None
    for ii in xrange(10000):
        sensor.update()
        curr = sensor.scan_index, sensor.safe_dist
        if prev != curr:
            if sensor.scan_index % 10 == 0:
                print '-----', sensor.scan_index, '-----'
                candidates = []
                for i, arr in enumerate(sensor.dist):
                    if min(arr) > 0 and (max(arr)-min(arr))/float(min(arr)) < MAX_TOLERANCE:
                        candidates.append((i, min(arr)*0.002))

                # find continuous interval
                if len(candidates) > 0:
                    results = []
                    base = candidates[0]
                    count = 0
                    for c in candidates[1:]:
                        if base[0] + 1 == c[0]:
                            count += 1
                        else:
                            results.append( (count, base) )
                            count = 0
                        base = c
                    results.append( (count, base) )

                    for r in results:
                        width = math.radians(r[0] + 1) * r[1][1]
                        if width < MAX_COLUMN_WIDTH:
                            print r[1],
                    print

            prev = curr

# vim: expandtab sw=4 ts=4 

