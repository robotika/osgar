#!/usr/bin/python
"""
  Velodyne VLP-16 wrapper
  usage:
       ./velodyne.py <task> [<metalog> [<F>]]
"""
import sys
import socket
import datetime
import struct
import time
import numpy as np

from apyros.metalog import MetaLog, disableAsserts


HOST = "192.168.1.201"
PORT = 2368

# bug in datasheet "Table 6. Laser ID"
LASER_ANGLES = [-15, 1, -13, 3, -11, 5, -9, 7, -7, 9, -5, 11, -3, 13, -1, 15]
NUM_LASERS = 16

def min_dist(data):
    mask = (data > 0)
    if np.any(mask):
        return np.min(data[mask]) * 0.002
    return None

class Velodyne:
    def __init__(self, metalog=None):
        if metalog is None:
            metalog = MetaLog()
        self.soc = metalog.createLoggedSocket("velodyne", headerFormat="<BBBI")
        self.soc.bind( ('',PORT) )
        self.metalog = metalog
        self.buf = ""
        self.time = None
        self.dist = np.zeros( (360, NUM_LASERS), dtype=np.uint16)
        self.scan_index = 0
        self.prev_azimuth = None
        self.safe_dist = None
        

    def parse(self, data):
        assert len(data) == 1206, len(data)
        while True:
            block, data = data[:100], data[100:]
            if len(data) < 100:
                assert len(data) == 6, len(data)
                break
            flag, azi = struct.unpack_from("<HH", block)
            assert flag == 0xEEFF, hex(flag)
            azimuth = azi/100.0
            if self.prev_azimuth is not None and azimuth < self.prev_azimuth:
                self.scan_index += 1
                self.safe_dist = min_dist(self.dist[160:200])
                if self.scan_index % 10 == 0:
                    print self.scan_index, self.safe_dist
            self.prev_azimuth = azimuth
            # H-distance (2mm step), B-reflectivity (0
            arr = struct.unpack_from('<' + "HB"*32, block, 4)
            for i in xrange(NUM_LASERS):
                self.dist[int(azimuth)][i] = arr[i*2]
        timestamp, factory = struct.unpack_from("<IH", data)
        assert factory == 0x2237, hex(factory)  # 0x22=VLP-16, 0x37=Strongest Return
        self.time = timestamp/1000000.0

    def update(self):
        while True:
            data = self.soc.recv(2000)
            if len(data) > 0:
                assert len(data) == 1206, len(data)
                break
        self.parse(data)


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print __doc__
        sys.exit(2)
    metalog=None
    if 'meta_' in sys.argv[1]:
        metalog = MetaLog(filename=sys.argv[1])
    elif len(sys.argv) > 2:
        metalog = MetaLog(filename=sys.argv[2])
    if len(sys.argv) > 2 and sys.argv[-1] == 'F':
        disableAsserts()

    sensor = Velodyne(metalog=metalog)
    for i in xrange(10000):
        sensor.update()

# vim: expandtab sw=4 ts=4 

