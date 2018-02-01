"""
  GPS Driver
"""

import sys
import os
import inspect

OSGAR_ROOT = os.path.realpath(os.path.abspath(os.path.join(os.path.split(inspect.getfile( inspect.currentframe() ))[0],"..")))
if OSGAR_ROOT not in sys.path:
    sys.path.insert(0, OSGAR_ROOT) # access to logger without installation


import serial
from threading import Thread, Event

from lib.logger import LogWriter, LogReader


def checksum(s):
    sum = 0
    for ch in s:
        sum ^= ch
    return b"%02X" % (sum)


def str2ms(s):
    'convert DDMM.MMMMMM string to arc milliseconds(int)'
    if s == b'':  # unknown position
        return None
    dm, frac = (b'0000' + s).split(b'.')
    return round((int(dm[:-2]) * 60 + float(dm[-2:] + b'.' + frac)) * 60000)


class GPS(Thread):
    def __init__(self, config, logger, output, name='gps'):
        Thread.__init__(self)
        self.setDaemon(True)
        self.should_run = Event()
        self.should_run.set()

        if 'port' in config:
            self.com = serial.Serial(config['port'], config['speed'])
            self.com.timeout = 0.01  # expects updates < 100Hz
        else:
            self.com = None
        self.logger = logger
        self.stream_id = config['stream_id']

        self.buf = b''
        self.output = output
        self.name = name

    @staticmethod
    def parse_line(line):
        assert line.startswith(b'$GNGGA') or line.startswith(b'$GPGGA'), line
        assert checksum(line[1:-3]) == line[-2:], (line, checksum(line[1:-3]))
        s = line.split(b',')
        return str2ms(s[4]), str2ms(s[2])

    @staticmethod
    def split_buffer(data):
        # in dGPS there is a block of binary data so stronger selection is required
        start = max(data.find(b'$GNGGA'), data.find(b'$GPGGA'))
        if start < 0:
            return data, b''
        end = data[start:-2].find(b'*')
        if end < 0:
            return data, b''
        return data[start+end+3:], data[start:start+end+3]

    def process(self, data):
        self.buf, line = self.split_buffer(self.buf + data)
        if line.startswith(b'$GNGGA') or line.startswith(b'$GPGGA'):
            coords = self.parse_line(line)
            if self.output:
                self.output(self.name, coords)
            return coords

    def run(self):
        while self.should_run.isSet():
            data = self.com.read(1024)
            if len(data) > 0:
                self.logger.write(self.stream_id, data)
                self.process(data)

    def request_stop(self):
        self.should_run.clear()


if __name__ == "__main__":
    if len(sys.argv) == 1:
        import time
        config = { 'port': 'COM5', 'speed': 4800, 'stream_id': 1 }
        log = LogWriter(prefix='gps-test')
        device = GPS(config, log, output=None)
        device.start()
        time.sleep(2)
        device.request_stop()
        device.join()
    else:
        filename = sys.argv[1]
        log = LogReader(filename)
        stream_id = 1
        device = GPS(config={'stream_id': stream_id}, logger=None, output=None)
        for timestamp, __, data in log.read_gen(only_stream_id=stream_id):
            print(data)
            out = device.process(data)
            if out is not None:
                print(out)

# vim: expandtab sw=4 ts=4
