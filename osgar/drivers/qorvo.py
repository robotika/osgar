"""
  Simple(minded) driver for Qorvo UWB modules
"""

from osgar.node import Node

# Note, that for ASCII shell communication add
# self.publish("raw", b'\r\r')  # switch to generic shell commands


class Qorvo(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('range', 'raw')
        self.buf = b''
        self.verbose = False

    def split_data(self, buf):
        if len(buf) < 2 or len(buf) < buf[1] + 2:
            return None, buf
        packet_len = buf[1] + 2
        return buf[:packet_len], buf[packet_len:]

    def on_raw(self, data):
        self.buf += data
        while True:
            packet, self.buf = self.split_data(self.buf)
            if packet is None:
                break
            if packet[0] == 0x40:
                # response
                assert len(packet) == 3, packet

    def on_timer(self, data):
        self.publish('raw', bytes([0x0C, 0x00]))  # request range reading


# vim: expandtab sw=4 ts=4
