"""
  Driver for CO2 sensor via USB
  Intelligent Infrared CO2 Module
    (Model: MH-Z19)
"""

import struct

from osgar.node import Node
from osgar.bus import BusShutdownException


class WinsenCO2(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('raw', 'co2')
        self.sleep_time = config.get('sleep')
        self._buf = b''
        self.errors = 0

    def create_packet(self):
        # request CO2 readings
        return b"\xff\x01\x86\x00\x00\x00\x00\x00\x79"

    def get_packet(self):
        """extract packet from internal buffer (if available otherwise return None"""
        data = self._buf
        if 0xFF not in data:
            self._buf = b''
            return None
        i = data.index(0xFF)
        data = data[i:]  # cut to the beginning of the packet
        size = 9
        if len(data) < size:
            self._buf = data
            return None
        ret, self._buf = data[:size], data[size:]
        checksum = sum(ret[1:]) & 0xFF
        if checksum != 0:
            self.errors += 1
            return None
        return ret

    def parse_CO2_packet(self, data):
        """
        Parse CO2 value
        """
        return struct.unpack_from('>H', data, 2)[0]

    def run(self):
        try:
            while True:
                dt, channel, data = self.listen()
                self.time = dt
                if channel == 'trigger':
                    self.publish('raw', self.create_packet())
                elif channel == 'raw':
                    self._buf += data
                    packet = self.get_packet()
                    if packet is not None:
                        value_CO2 = self.parse_CO2_packet(packet)
                        if value_CO2 is None:
                            print(packet)
                        else:
                            self.publish('co2', value_CO2)

        except BusShutdownException:
            pass

        assert self.errors == 0, self.errors  # checksum

# vim: expandtab sw=4 ts=4
