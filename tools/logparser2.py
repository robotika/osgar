#!/usr/bin/python
"""
  Log parser - 2nd generation (timestamps based)
  usage:
       ./logparser2.py <metalog>
"""
import ast
import math
import os
import struct
import sys
from zipfile import ZipFile


def timestamps_gen(id, stream):
    if id == 'can': # binary log, not implemented
        return
    if id == 'timestamps':
        for line in stream:
            line = ast.literal_eval(line)
            assert len(line) == 3
            yield float(line[0]), 'can', line[1:]
    for i, line in enumerate(stream):
        if i % 2 == 1:
            data = ast.literal_eval(line)
            yield timestamp, id, data
        else:
            vals = line.split()
            if len(vals) != 2: # only at the end of file
                return
            timestamp = float(vals[1])


def sensor_gen(meta_filename, selected=None):
    """
      Yield timestamp ordered data from all selected sensors.
      Use None for all available sensors
    """
    meta_dir = os.path.dirname(meta_filename)
    filenames = {}
    with open(meta_filename, 'rb') as meta:
        for line in filter(lambda a: ':' in a, meta): # skip unknown lines
            sensor, filepath = line.strip().split(':')
            filename = os.path.basename(filepath)
            filenames[sensor] = os.path.join(meta_dir, filename)

    if selected is not None:
        # delete filenames not in selected
        for key in filenames.viewkeys() ^ selected:
            del filenames[key]

    # zip multiple generators
    data = []
    for sensor, filename in filenames.iteritems():
        with open(filename, 'rb') as stream:
            data.extend(timestamps_gen(sensor, stream))

    data.sort()
    for a in data:
        yield a


def sensor_gen_zip(filepath, selected=None):
    with ZipFile(filepath) as zip:
        filenames = zip.namelist()
        meta_filename = [name for name in filenames if name.startswith('meta_')][0]

        filenames = {}
        with zip.open(meta_filename) as meta:
            for line in filter(lambda a: ':' in a, meta): # skip unknown lines
                sensor, filepath = line.strip().split(':')
                filenames[sensor] = os.path.basename(filepath)

        if selected is not None:
            # delete filenames not in selected
            for key in filenames.viewkeys() ^ selected:
                del filenames[key]

        # zip multiple generators
        data = []
        for sensor, filename in filenames.iteritems():
            with zip.open(filename) as stream:
                data.extend(timestamps_gen(sensor, stream))

        data.sort()
        for a in data:
            yield a

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(2)
    for timestamp, sensor, data in sensor_gen_zip(sys.argv[1]):
        print(timestamp, sensor, data[:10])

# vim: expandtab sw=4 ts=4 
