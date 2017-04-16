#!/usr/bin/python
"""
  Log parser - 2nd generation (timestamps based)
  usage:
       ./meta2view.py <metalog>
"""
import math
import os
import struct
import sys


def timestamps_gen(filename):
    timestamp = None
    for i, line in enumerate(open(filename)):
        if i % 2 == 1:
            data = eval(line)
            yield timestamp, data
        else:
            if len(line.split()) > 1:
                timestamp = float(line.split()[1])
            else:
                timestamp = None


def sensor_gen(meta_filename, selected=None):
    """
      Yield timestamp ordered data from all selected sensors.
      Use None for all available sensors
    """
    meta_dir = os.path.split(meta_filename)[0]
    filenames = {}
    for line in open(meta_filename):
        print(line)
        if ':' in line:
            sensor = line.split(':')[0]
            filename = os.path.split(line.split()[1])[1]
            filenames[sensor] = os.path.join(meta_dir, filename)

    print filenames
    if selected is not None:
        # filter filenames via selected
        tmp = {}
        for key in selected:
            if key in filenames:
                tmp[key] = filenames[key]
        filenames = tmp

    # zip multiple generators
    assert len(filenames) == 1, filenames  # not implemented yet
    sensor, filename = filenames.items()[0]

    for timestamp, data in timestamps_gen(filename):
        yield timestamp, sensor, data

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(2)
    for timestamp, sensor, data in sensor_gen(sys.argv[1], ['camera']):
        pass

# vim: expandtab sw=4 ts=4 

