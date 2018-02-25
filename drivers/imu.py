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
from threading import Thread

from lib.logger import LogWriter, LogReader
from drivers.bus import BusShutdownException
from drivers.gps import checksum


def parse_line(line):
    assert line.startswith(b'$VNYMR'), line
    assert b'*' in line, line
    s = line.split(b'*')[0].split(b',')
    assert len(s) == 13, s
    arr = [float(x) for x in s[1:]]
    return arr[:3], arr[3:6], arr[6:9], arr[9:]


class IMU(Thread):
    def __init__(self, config, bus):
        Thread.__init__(self)
        self.setDaemon(True)

        self.bus = bus
        self.buf = b''

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

    def process_packet(self, line):
        if line.startswith(b'$VNYMR'):
            result = parse_line(line)
            return result
        return None

    def process_gen(self, data):
        self.buf, packet = self.split_buffer(self.buf + data)
        while len(packet) > 0:
            ret = self.process_packet(packet)
            if ret is not None:
                yield ret
            # now process only existing (remaining) buffer
            self.buf, packet = self.split_buffer(self.buf)  

    def run(self):
        try:
            while True:
                packet = self.bus.listen()
                dt, __, data = packet
                for out in self.process_gen(data):
                    assert out is not None
                    self.bus.publish('orientation', out)
        except BusShutdownException:
            pass

    def request_stop(self):
        self.bus.shutdown()


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
