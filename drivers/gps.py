"""
  GPS Driver
"""

import sys
import os
import inspect

OSGAR_ROOT = os.path.realpath(os.path.abspath(os.path.join(os.path.split(inspect.getfile( inspect.currentframe() ))[0],"..")))
if OSGAR_ROOT not in sys.path:
    sys.path.insert(0, OSGAR_ROOT) # access to logger without installation

from threading import Thread
import numpy as np

from lib.logger import LogWriter, LogReader
from drivers.bus import BusShutdownException


GPS_MSG_DTYPE = [('lon', 'i4'), ('lat', 'i4')]
INVALID_COORDINATES = np.array((0x7FFF, 0x7FFF), dtype=GPS_MSG_DTYPE)


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
    def __init__(self, config, bus):
        Thread.__init__(self)
        self.setDaemon(True)

        self.bus = bus
        self.stream_id = config['stream_id']

        self.buf = b''

    @staticmethod
    def parse_line(line):
        assert line.startswith(b'$GNGGA') or line.startswith(b'$GPGGA'), line
        assert checksum(line[1:-3]) == line[-2:], (line, checksum(line[1:-3]))
        s = line.split(b',')
        coord = str2ms(s[4]), str2ms(s[2])
        if coord == (None, None):
            return INVALID_COORDINATES
        return np.array(coord, dtype=GPS_MSG_DTYPE)

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
            return coords
        return None

    def run(self):
        try:
            while True:
                packet = self.bus.listen()  # there should be some timeout and in case of failure send None
                dt, __, data = packet
                out = self.process(data)
                if out is not None:
                    self.bus.publish(self.stream_id, out)
        except BusShutdownException:
            pass

    def request_stop(self):
        self.bus.shutdown()


def print_output(packet):
    print(packet)


# TODO move somewhere else ...
class LogWriterEx(LogWriter):
    def write(self, stream_id, data):
        try:
            bytes_data = data.tobytes()
        except AttributeError:
            bytes_data = data
        LogWriter.write(self, stream_id, bytes_data)


if __name__ == "__main__":
    if len(sys.argv) == 1:
        import time
        from drivers.logserial import LogSerial
        from drivers.bus import BusHandler

        config_serial = {'port': 'COM5', 'speed': 4800, 'stream_id': 1}
        config_gps = {'stream_id': 2}
        log = LogWriterEx(prefix='gps-test')
        device = GPS(config_gps, bus=BusHandler(log, out={2:[]}))
        device0 = LogSerial(config_serial, bus=BusHandler(log, out={1:[(device.bus.queue, 1)]}))
        device.start()
        device0.start()
        time.sleep(2)
        device.request_stop()
        device0.request_stop()
        device.join()
        device0.join()
    else:
        import ast

        filename = sys.argv[1]
        log = LogReader(filename)
        stream_id_in = 1
        stream_id_out = 2
        device = GPS(config={'stream_id': stream_id_in}, bus=None)
        arr = []
        for timestamp, stream_id, data in log.read_gen(only_stream_id=[stream_id_in, stream_id_out]):
            if stream_id == stream_id_in:
                arr.append(data)
            elif stream_id == stream_id_out:
                ref = np.frombuffer(data, dtype=GPS_MSG_DTYPE)
                for i, data in enumerate(arr):
                    out = device.process(data)
                    if out is not None:
                        assert out == ref, (out, ref)
                        print(out)
                        arr = arr[i:]
                        break
                else:
                    assert False, ref  # output was note generated
            else:
                assert False, stream_id  # unexpected stream

# vim: expandtab sw=4 ts=4
