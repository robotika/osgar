"""
  Handle data from VanJee Lidar WLR-719C (4 lines)
"""
import struct
import math

from osgar.node import Node


class VanJeeLidar(Node):
    def __init__(self, config, bus):
        bus.register('raw', 'xyz', 'scan')
        super().__init__(config, bus)

    def on_raw(self, data):
        pass

# vim: expandtab sw=4 ts=4
