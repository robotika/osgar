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
        pass

    def run(self):
        magic_packet = b'\xFF\xAA\x00\x1E\x00\x00\x00\x00\x00\x00\x01\x01\x19\x0C\x00\x00\x00\x00\x00\x00\x00\x00\x05\x03\x00\x00\x00\x00\x00\x00\x00\x0D\xEE\xEE'
        self.publish('raw', magic_packet)
        try:
            while True:
                self.update()
        except BusShutdownException:
            pass


# vim: expandtab sw=4 ts=4
