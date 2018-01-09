"""
  Simple IMU data analysis
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
from drivers.gps import checksum


def parse_line(line):
    assert line.startswith(b'$VNYMR'), line
    assert b'*' in line, line
    s = line.split(b'*')[0].split(b',')
    assert len(s) == 13, s
    arr = [float(x) for x in s[1:]]
    return arr[:3], arr[3:6], arr[6:9], arr[9:]


class IMU(Thread):
    def __init__(self, config, logger, output, name='imu'):
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


    # Copy & Paste from gps.py - refactor!
    @staticmethod
    def split_buffer(data):
        start = data.find(b'$')
        if start < 0:
            return data, b''
        end = data[start:-2].find(b'*')
        if end < 0:
            return data, b''
        return data[start+end+3:], data[start:start+end+3]

    def process(self, data):
        self.buf, line = self.split_buffer(self.buf + data)
        if line.startswith(b'$VNYMR'):
            result = parse_line(line)
            if self.output:
                self.output(self.name, result)
            return result

    def run(self):
        while self.should_run.isSet():
            data = self.com.read(1024)
            if len(data) > 0:
                self.logger.write(self.stream_id, data)
                self.process(data)

    def request_stop(self):
        self.should_run.clear()


if __name__ == "__main__":
    import io
    import sys
    import matplotlib.pyplot as plt
    
    arr = []
    for line in io.open(sys.argv[1]):
        angle, mag, acc, gyro = parse_line(line)
        arr.append(angle)

    plt.plot(arr, 'o-', linewidth=2)
    plt.show()

# vim: expandtab sw=4 ts=4
