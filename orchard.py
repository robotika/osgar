#!/usr/bin/python
"""
  Analysis of Velodyne VLP-16 data - orchard navigation
  usage:
       ./orchard.py <metalog>
"""
import sys
import math
from apyros.metalog import MetaLog
from velodyne import Velodyne

def analyse_alley(raw_data):
    s = ''
    for i in list(range(270, 360, 5)) + list(range(0, 90, 5)):
        dist = raw_data[i:i+5][:,1::2]  # use only part related +/- 7 deg
        mask = dist > 0
        if mask.max():
            d = 0.002*dist[mask].min()
        else:
            d = 10.0

        ch = ' '
        if d < 2.0:
            ch = 'X'
        elif d < 4.0:
            ch = 'x'
        s += ch
    return s


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(2)
    assert 'meta_' in sys.argv[1], sys.argv[1]
    metalog = MetaLog(filename=sys.argv[1])
    sensor = Velodyne(metalog=metalog)


    prev = None
    while True:
        sensor.update()
        curr = sensor.scan_index, sensor.safe_dist
        if prev != curr:
            if sensor.scan_index % 5 == 0 and sensor.scan_index > 0:
#                print '-----', sensor.scan_index, '-----'
                print(analyse_alley(sensor.dist))
            prev = curr

# vim: expandtab sw=4 ts=4 

