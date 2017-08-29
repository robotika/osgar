#!/usr/bin/env python
"""
  parse debug laser data
    ./parse_laserraw.py <laserraw filename>
"""

import sys
import struct


def parse_laserraw(filename):
    version = 1
    block_header_size = 4
    timestamp = None
    with open(filename, 'rb') as f:
        buf = f.read(4)
        if len(buf) == 4:
            magic, version = struct.unpack('HH', buf)
            if magic != 0:  # old version was first sending data, no header
                assert magic == 0xf472, hex(magic)
                assert version == 2, version # with timestamps
                buf = f.read(6) # read extra timestamp when laser was connected
                time_sec, time_frac = struct.unpack('IH', buf)
                start_time = time_sec + float(time_frac)/0x10000
                start_time = 0 # hack to have absolute time for now
                block_header_size = 6 + 4  # timestamp + dir + len
                buf = f.read(block_header_size)
        while len(buf) == block_header_size:
            if version == 1:
                io, size = struct.unpack('HH', buf)
            else:
                time_sec, time_frac, io, size = struct.unpack('IHHH', buf)
                timestamp = time_sec + float(time_frac)/0x10000
            assert io in [0, 1]
            data = f.read(size)
            if timestamp is not None:
                print '%.03f' % (timestamp-start_time), data[:60]
            else:
                print data
            buf = f.read(block_header_size)


if __name__ == "__main__": 
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(2)

    parse_laserraw(sys.argv[1])

# vim: expandtab sw=4 ts=4 

