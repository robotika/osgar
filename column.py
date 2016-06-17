#!/usr/bin/python
"""
  Analysis of Velodyne VLP-16 data - column detection
  usage:
       ./column.py <metalog>
"""
import sys
from apyros.metalog import MetaLog
from velodyne import Velodyne


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print __doc__
        sys.exit(2)
    assert 'meta_' in sys.argv[1], sys.argv[1]
    metalog = MetaLog(filename=sys.argv[1])
    sensor = Velodyne(metalog=metalog)


    prev = None
    for i in xrange(1000):
        sensor.update()
        curr = sensor.scan_index, sensor.safe_dist
        if prev != curr:
            if sensor.scan_index % 10 == 0:
                print curr
            prev = curr

# vim: expandtab sw=4 ts=4 

