"""
  Simple(minded) driver for Qorvo UWB modules
"""
import struct

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
            cmd = packet[0]
            if cmd == 0x40:
                # response start and end
                assert len(packet) == 3, packet
            elif cmd == 0x41:
                # position
                assert len(packet) == 2 + 13, packet
                # X, Y, Z + quality
                assert packet[-1] == 0, packet[-1]  # i.e. invalid position
            elif cmd == 0x49:
                # range distance
                num_distances = packet[2]
                dist_arr = []
                if num_distances > 0:
                    for i in range(num_distances):
                        uwb_addr, dist, quality, x, y, z, pos_quality = struct.unpack_from(
                            '<HIBiiiB',packet, 3 + i*20)
                        dist_arr.append([uwb_addr, dist])
                self.publish('range', dist_arr)

    def on_timer(self, data):
        self.publish('raw', bytes([0x0C, 0x00]))  # request range reading


# vim: expandtab sw=4 ts=4
