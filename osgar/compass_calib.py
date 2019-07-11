"""
  Compass calibration from log file
"""

import argparse
import struct

import matplotlib.pyplot as plt

from osgar.logger import LogReader, lookup_stream_id
from osgar.lib.serialize import deserialize


COMPASS_ADDR = 0x1E


def compass_calib(filename):
    name = 'i2c.i2c'
    stream_id = lookup_stream_id(filename, name)

    val = 0x8000
    minx, maxx, miny, maxy = val, -val, val, -val

    record = []
    with LogReader(filename) as log:
        for timestamp, __, data in log.read_gen(stream_id):
            data = deserialize(data)
            addr, rw, arr = data
            if addr == COMPASS_ADDR:
                assert len(arr) == 6, arr
                x, z, y = struct.unpack('>hhh', bytes(arr))  # axis Y and Z swapped in orig
                record.append((x, y, z))
                minx = min(minx, x)
                maxx = max(maxx, x)
                miny = min(miny, y)
                maxy = max(maxy, y)

    arr = record
    x = [x for (x, _, _) in arr]
    y = [y for (_, y, _) in arr]
    plt.plot(x, y, 'o-', linewidth=2)
    
    return (minx+maxx)//2, (miny+maxy)//2, [minx, maxx, miny, maxy]

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Calibrate compass from log')
    parser.add_argument('logfile', help='recorded log file')
    args = parser.parse_args()

    print(compass_calib(args.logfile))
    plt.show()


# vim: expandtab sw=4 ts=4
