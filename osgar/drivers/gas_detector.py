"""
  Driver for CO2 sensor via USB, using communication protocol similar to Robik
"""

import struct

from osgar.node import Node
from osgar.bus import BusShutdownException


class MeasureCO2(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('raw', 'co2')
        self.sleep_time = config.get('sleep')
        self._buf = b''

    def query_version(self):
        ret = bytes([0, 0, 3, 0x1, 0x01])
        checksum = sum(ret) & 0xFF
        return ret + bytes([(256-checksum) & 0xFF])

    def create_packet(self):
        # request CO2 readings
        return bytes([0x00,0x00,0x03,0x01,0xC0,0x3C])

    def get_packet(self):
        """extract packet from internal buffer (if available otherwise return None"""
        data = self._buf
        if len(data) < 3:
            return None
        high, mid, low = data[:3]  # 24bit packet length (big endian int)
        assert high == 0, high  # all messages < 65535 bytes
        size = 256 * mid + low + 3  # counting also 3 bytes of packet length header
        if len(data) < size:
            return None
        ret, self._buf = data[:size], data[size:]
        checksum = sum(ret) & 0xFF
        assert checksum == 0, checksum  # checksum error
        return ret

    def parse_CO2_packet(self, data):
        """
        Parse cortexpilot sensors status message
        """
        # expects already validated single sample with 3 bytes length prefix
        #   and checksum at the end
        high, mid, low = data[:3]
        addr, cmd = data[3:5]
        if cmd != 0xC0:
            return None

        assert high == 0, high  # fixed packet size 2*256+89 bytes
        assert mid == 0, mid
        assert low == 7, low
        assert addr == 1, addr
        offset = 5  # payload offset

        return struct.unpack_from('<H', data, offset)[0]

    def run(self):
        try:
            self.publish('raw', self.query_version())
            while True:
                dt, channel, data = self.listen()
                self.time = dt
                if channel == 'raw':
                    self._buf += data
                    packet = self.get_packet()
                    if packet is not None:
                        value_CO2 = self.parse_CO2_packet(packet)
                        if value_CO2 is None:
                            print(packet)
                        else:
                            self.publish('co2', value_CO2)

                        if self.sleep_time is not None:
                            self.sleep(self.sleep_time)
                        self.publish('raw', self.create_packet())

        except BusShutdownException:
            pass

# vim: expandtab sw=4 ts=4
