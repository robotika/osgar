#!/usr/bin/python
"""
  Velodyne VLP-16 wrapper
  usage:
       ./velodyne.py <task|thread> [<metalog> [<F>]]
"""
import sys
import socket
from datetime import datetime, timedelta
import struct
import time
import numpy as np

from apyros.metalog import MetaLog, disableAsserts
from threading import Thread,Event,Lock


HOST = "192.168.1.201"
PORT = 2368

# bug in datasheet "Table 6. Laser ID"
LASER_ANGLES = [-15, 1, -13, 3, -11, 5, -9, 7, -7, 9, -5, 11, -3, 13, -1, 15]
NUM_LASERS = 16

EXPECTED_PACKET_TIME = 0.001327  # valid only in "the strongest return mode"
EXPECTED_SCAN_DURATION = 0.1

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
        self.last_blocked = None
        self.dist = np.zeros( (360, NUM_LASERS), dtype=np.uint16)
        self.scan_index = 0
        self.prev_azimuth = None
        self.safe_dist = None
        

    def parse(self, data):
        assert len(data) == 1206, len(data)
        timestamp, factory = struct.unpack_from("<IH", data, offset=1200)
        assert factory == 0x2237, hex(factory)  # 0x22=VLP-16, 0x37=Strongest Return
        time = timestamp/1000000.0
        if self.time is not None:
            lost_packets = int(round((time - self.time)/EXPECTED_PACKET_TIME)) - 1
        else:
            lost_packets = 0
        self.time = time
        if lost_packets > 0 and (self.last_blocked is None or self.time > self.last_blocked + EXPECTED_SCAN_DURATION):
            self.last_blocked = self.time + EXPECTED_SCAN_DURATION
            self.scan_index += 1
            print "DROPPED index", self.scan_index
        if self.last_blocked is not None and self.time < self.last_blocked:
            return  # to catch up-to-date packets again ...

        for offset in xrange(0, 1200, 200):  # skip every second packet (we need 1deg resolution input 0.4)
            flag, azi = struct.unpack_from("<HH", data, offset)
            assert flag == 0xEEFF, hex(flag)
            azimuth = azi/100.0
            if self.prev_azimuth is not None and azimuth < self.prev_azimuth:
                self.scan_index += 1
# rotated 180deg -> this is back now:     self.safe_dist = min_dist(self.dist[160:200])
                self.safe_dist = (min_dist(self.dist[340:360]), min_dist(self.dist[0:20]))  # (left, right)
            self.prev_azimuth = azimuth
            # H-distance (2mm step), B-reflectivity (0
            arr = struct.unpack_from('<' + "HB"*32, data, offset + 4)
            for i in xrange(NUM_LASERS):
                self.dist[int(azimuth)][i] = arr[i*2]


    def update(self):
        while True:
            data = self.soc.recv(2000)
            if len(data) > 0:
                assert len(data) == 1206, len(data)
                break
        self.parse(data)

# TODO general wrapper for threaded sources
class VelodyneThread(Thread):
    def __init__(self, sensor):
        Thread.__init__(self)
        self.setDaemon(True)
        self.lock = Lock()
        self.shouldIRun = Event()
        self.shouldIRun.set()
        self.sensor = sensor
        self._result = None
        self._last_index = None

    def run(self):
        while self.shouldIRun.isSet():
            self.sensor.update()
            if self._last_index != self.sensor.scan_index:
                self.lock.acquire()
                self._result = self.sensor.scan_index, self.sensor.safe_dist
                self._last_index = self.sensor.scan_index
                self.lock.release()

    def scan_safe_dist(self):
        self.lock.acquire()
        ret = self._result  # copy?
        self.lock.release()
        return ret

    def requestStop(self):
        self.shouldIRun.clear()


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
    if sys.argv[1] == 'thread':
        thr = VelodyneThread(sensor)
        start_time = datetime.now()
        thr.start()
        prev = None
        while datetime.now() - start_time < timedelta(seconds=3.0):
            curr = thr.scan_safe_dist()
            if prev != curr:
                print curr
            prev = curr
        thr.requestStop()
        thr.join()
    else:
        prev = None
        for i in xrange(10000):
            sensor.update()
            curr = sensor.scan_index, sensor.safe_dist
            if prev != curr:
                if sensor.scan_index % 10 == 0:
                    print curr
            prev = curr

# vim: expandtab sw=4 ts=4 

