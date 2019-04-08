"""
  DARPA E-stop Tier level 2
"""
from osgar.node import Node
from osgar.bus import BusShutdownException

ATIS_FRAME_PACKET = bytes([0x7E, 0x00, 0x04, 0x08, 0x01, 0x49, 0x53, 0x5A])
NORMAL_OPERATION_PACKET = bytes([0x7E, 0x00, 0x0B, 0x88, 0x01, 0x49, 0x53, 0x00,
                                 0x01, 0x00, 0x02, 0x00, 0x00, 0x00, 0xD7])
EMERGENCY_STOP_PACKET = bytes([0x7E, 0x00, 0x0B, 0x88, 0x01, 0x49, 0x53, 0x00,
                               0x01, 0x00, 0x02, 0x00, 0x00, 0x02, 0xD5])
HEADER = bytes([0x7E, 0x00, 0x0B, 0x88, 0x01, 0x49, 0x53])
PACKET_SIZE = 15  # including header and checksum


class EStop(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        self._buf = b''

    def update(self):  # hack, this method should be called run instead!
        channel = super().update()  # define self.time
        assert channel == "raw", channel
        self._buf += self.raw

        packet = None
        while HEADER in self._buf and self._buf.index(HEADER) + PACKET_SIZE <= len(self._buf):
            i = self._buf.index(HEADER)
            packet = self._buf[i:i+PACKET_SIZE]
            self._buf = self._buf[i+PACKET_SIZE:]       

        if packet is not None:
            print('packet:', len(packet), packet.hex())
            if packet[-2] == 0x02:
                self.publish('emergency_stop', True)
            self.sleep(1.0)
            self.publish('raw', ATIS_FRAME_PACKET)

    def run(self):
        try:
            self.publish('raw', ATIS_FRAME_PACKET)
            while True:
                self.update()
        except BusShutdownException:
            pass

# vim: expandtab sw=4 ts=4
