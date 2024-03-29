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

MASTER_STOP = bytes.fromhex('7E 00 10 17 01 00 00 00 00 00 00 FF FF FF FE 03 44 31 05 6F')


class EStop(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('emergency_stop', 'raw')
        self._buf = b''
        self.master = config.get('master', False)

    def on_raw(self, data):
        self._buf += data
        packet = None
        while HEADER in self._buf and self._buf.index(HEADER) + PACKET_SIZE <= len(self._buf):
            i = self._buf.index(HEADER)
            packet = self._buf[i:i+PACKET_SIZE]
            self._buf = self._buf[i+PACKET_SIZE:]       

        if packet is not None:
            print('packet:', len(packet), packet.hex())
            if packet[-2] == 0x02:
                self.publish('emergency_stop', True)
                self.request_stop()
            self.sleep(1.0)
            if self.master:
                self.publish('raw', MASTER_STOP)
            else:
                self.publish('raw', ATIS_FRAME_PACKET)

    def run(self):
        try:
            if self.master:
                self.publish('raw', MASTER_STOP)
            else:
                self.publish('raw', ATIS_FRAME_PACKET)
            while True:
                self.update()
        except BusShutdownException:
            pass

# vim: expandtab sw=4 ts=4
