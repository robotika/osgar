#!/usr/bin/python
"""
  Plot data
  usage:
       ./plot.py <text data>
"""
import sys
import math
import struct
import os.path

import matplotlib.pyplot as plt

from osgar.logger import LogReader, lookup_stream_id
from osgar.lib.serialize import deserialize


def get_arr0(filename):
    arr = []
    for line in open(filename):
        if 'xENC' in line:
            prefix_cmd, t, x, y = line.split()
            arr.append((t, (int(x), int(y))))
        if 'WHEEL' in line:
            prefix_cmd, t, angle, desired = line.split()
            if desired != 'None':
                arr.append((t, (int(angle), float(desired))))
            else:
                arr.append((t, (int(angle), 0)))
        if 'xSPEED' in line:
            prefix_cmd, t, raw, avr = line.split()
            arr.append((t, (raw, avr)))
        if 'xGAS' in line:
            prefix_cmd, gas = line.split()
            arr.append((len(arr), int(gas)))
        if 'xSYNC' in line:
            gas = line.split()[-1]
            if gas != '0':
                arr.append((len(arr), float(gas)))
    return arr

def draw(arr, title=None):
#    plt.plot(arr, 'o-', linewidth=2)
    x = [x for (x, _) in arr]
    y = [y for (_, y) in arr]
    plt.plot(x, y, 'o-', linewidth=2)
    if title is not None:
        plt.title(title)

#    z = []
#    for i in xrange(len(y)):
#        z.append(sum(y[i-50:i+50])/100.0)
#    plt.plot(x, z, 'o-', linewidth=2)

    plt.xlabel('time (sec)', fontsize = 12)
    plt.ylabel('power (Volts)', fontsize = 12)
    plt.show()


def get_arr(filename):
    only_stream = lookup_stream_id(filename, 'can.can')
    
    arr = []
    with LogReader(filename) as log:
        for timestamp, stream_id, data in log.read_gen(only_stream):
            packet = deserialize(data)
            msg_id = ((packet[0]) << 3) | (((packet[1]) >> 5) & 0x1f)
            if msg_id == 0x18B:
                arr.append((timestamp.total_seconds(),
                            struct.unpack('<H', packet[2:])[0]/100.0))
    return arr

def get_laser_arr(filename, index):
    only_stream = lookup_stream_id(filename, 'lidar.scan')
    arr = []
    with LogReader(filename) as log:
        for ind, row in enumerate(log.read_gen(only_stream)):
            if ind < index:
                continue
            timestamp, stream_id, data = row
            data = deserialize(data)
            for i, dist in enumerate(data):
                arr.append((i, dist))
            break
    return arr

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description='Plot log data')
    parser.add_argument('filename', help='input log file')
    parser.add_argument('--laser', type=int, help='display laser data')
    args = parser.parse_args()

    if args.laser is None:
        arr = get_arr(args.filename)
    else:
        arr = get_laser_arr(args.filename, args.laser)
    draw(arr, title=os.path.basename(args.filename))


# vim: expandtab sw=4 ts=4 

