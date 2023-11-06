"""
  Handle data from VanJee Lidar WLR-719C (4 lines)
"""
import struct
import math

from osgar.node import Node
from osgar.bus import BusShutdownException


class VanJeeLidar(Node):
    def __init__(self, config, bus):
        bus.register('raw', 'xyz', 'scan', 'scan10', 'scanup')
        super().__init__(config, bus)
        self.last_frame = None  # not defined
        self.points = []

    def on_raw(self, data):
        assert len(data) in [34, 1384], len(data)
        if len(data) != 1384:
            return
        assert data[:2] == b'\xFF\xAA', data[:2].hex()
        length = struct.unpack_from('>H', data, 2)[0]
        assert length == 1380, length
        frame = struct.unpack_from('>H', data, 4)[0]
        if self.last_frame is not None:
            assert self.last_frame + 1 == frame, (self.last_frame, frame)
        self.last_frame = frame
        assert data[6:8] == bytes.fromhex('0000'), data[6:10].hex()  # reserved
        # reserved [8:10]  - variable
        assert data[11:14] == bytes.fromhex('02 190C'), data[11:14].hex()
        # reserved [14:16]
        assert data[16:18] == bytes.fromhex('01 04'), data[16:18].hex()
        bank, motor_speed = struct.unpack_from('>BH', data, 18)
        assert 1 <= bank <= 16, bank
        assert 38000 <= motor_speed <= 39300, motor_speed
        if bank == 1:
            self.points = []  # reset last scan

        assert data[-2:] == b'\xEE\xEE', data[-2:].hex()
        self.points.extend(struct.unpack_from('>' + 'BH' * 450, data, 21))
        if bank == 16:
            if len(self.points) == 2*7200:
                scan = self.points[5::8]  # -10, -5, 0, 0.3
                self.publish('scan', scan)
                scan10 = self.points[1::8]  # -10, -5, 0, 0.3
                self.publish('scan10', scan10)
                scanup = self.points[7::8]  # -10, -5, 0, 0.3
                self.publish('scanup', scanup)
            else:
                print(self.time, f'Incomplete scan - only {len(self.points)//2} points out of 7200')  # intensity, dist in mm

    def run(self):
        magic_packet = b'\xFF\xAA\x00\x1E\x00\x00\x00\x00\x00\x00\x01\x01\x19\x0C\x00\x00\x00\x00\x00\x00\x00\x00\x05\x03\x00\x00\x00\x00\x00\x00\x00\x0D\xEE\xEE'
        self.publish('raw', magic_packet)
        try:
            while True:
                self.update()
        except BusShutdownException:
            pass


# vim: expandtab sw=4 ts=4
