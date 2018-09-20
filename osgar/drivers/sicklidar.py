"""
  SICK Tim571 LIDAR Driver
"""

import time
from threading import Thread

from osgar.bus import BusShutdownException


STX = b'\x02'
ETX = b'\x03'


class SICKLidar(Thread):
    def __init__(self, config, bus):
        Thread.__init__(self)
        self.setDaemon(True)

        self.bus = bus
        self.buf = b''
        self.sleep = config.get('sleep')

    @staticmethod
    def parse_raw_data(raw_data):
        data = raw_data.split()
        assert len(data) in [854, 846], len(data)
        assert data[1] == b'LMDscandata', data[:2]
        timestamp = int(data[9], 16)  # TODO verify
        assert data[20] == b'DIST1', data[20]
        scan_start = 24  # TODO verify (prev version was 26)
        scan_size = 270 * 3 + 1
        scan_end = scan_start + scan_size
        dist = [int(x, 16) for x in data[scan_start:scan_end]]
        return dist

    def process_packet(self, packet):
        if packet.startswith(STX) and packet.endswith(ETX):
            return self.parse_raw_data(packet)
        return None

    def split_buffer(self, data):
        i = data.find(ETX)
        if i >= 0:
            return data[i + 1:], data[:i + 1]
        return data, b''

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
            self.bus.publish('raw', STX + b'sRN LMDscandata' + ETX)
            while True:
                packet = self.bus.listen()
                dt, __, data = packet
                for out in self.process_gen(data):
                    assert out is not None
                    self.bus.publish('scan', out)
                    if self.sleep is not None:
                        time.sleep(self.sleep)  # TODO skip in replay
                    self.bus.publish('raw', STX + b'sRN LMDscandata' + ETX)
        except BusShutdownException:
            pass

    def request_stop(self):
        self.bus.shutdown()

