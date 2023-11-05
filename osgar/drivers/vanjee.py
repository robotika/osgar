"""
  Handle data from VanJee Lidar WLR-719C (4 lines)
"""
import struct
import math

from osgar.node import Node
from osgar.bus import BusShutdownException


class VanJeeLidar(Node):
    def __init__(self, config, bus):
        bus.register('raw', 'xyz', 'scan')
        super().__init__(config, bus)

    def on_raw(self, data):
        assert len(data) in [34, 1384], len(data)
        if len(data) != 1384:
            return
        assert data[:2] == b'\xFF\xAA', data[:2].hex()
        length = struct.unpack_from('>H', data, 2)[0]
        assert length == 1380, length
        frame = struct.unpack_from('>H', data, 4)[0]
        assert data[6:8] == bytes.fromhex('0000'), data[6:10].hex()  # reserved
        # reserved [8:10]  - variable
        assert data[11:14] == bytes.fromhex('02 190C'), data[11:14].hex()
        # reserved [14:16]
        assert data[16:18] == bytes.fromhex('01 04'), data[16:18].hex()
        bank, motor_speed = struct.unpack_from('>BH', data, 18)
        assert 1 <= bank <= 16, bank
        assert 38000 <= motor_speed <= 39300, motor_speed

        assert data[-2:] == b'\xEE\xEE', data[-2:].hex()
        points = struct.unpack_from('>' + 'BH' * 450, data, 21)
#        assert 0, points[:20]

    def run(self):
        magic_packet = b'\xFF\xAA\x00\x1E\x00\x00\x00\x00\x00\x00\x01\x01\x19\x0C\x00\x00\x00\x00\x00\x00\x00\x00\x05\x03\x00\x00\x00\x00\x00\x00\x00\x0D\xEE\xEE'
        self.publish('raw', magic_packet)
        try:
            while True:
                self.update()
        except BusShutdownException:
            pass


# vim: expandtab sw=4 ts=4
