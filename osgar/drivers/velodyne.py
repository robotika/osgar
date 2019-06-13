"""
  Parse Velodyne VLP-16 data
"""
import struct
import math

from osgar.node import Node


def parse_packet(packet):
    return []


class Velodyne(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)

    def update(self):
        channel = super().update()
        assert channel == 'raw', channel
        self.publish('xyz', [[1, 2, 3], [4, 5, 6]])

# vim: expandtab sw=4 ts=4
