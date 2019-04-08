"""
  DARPA E-stop Tier level 2
"""
from osgar.node import Node
from osgar.bus import BusShutdownException

ATIS_FRAME_PACKET = bytes([0x7E, 0x00, 0x04, 0x08, 0x01, 0x49, 0x53, 0x5A])

class EStop(Node):
    def update(self):  # hack, this method should be called run instead!
        channel = super().update()  # define self.time
        assert channel == "raw", channel

        print("DETECTED:", self.raw)

    def run(self):
        try:
            self.publish('raw', ATIS_FRAME_PACKET)
            while True:
                self.update()
                self.sleep(1.0)
                self.publish('raw', ATIS_FRAME_PACKET)
        except BusShutdownException:
            pass

# vim: expandtab sw=4 ts=4
