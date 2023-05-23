"""
  RTK filter (reduce GPS NMEA to 1Hz for query to server)
"""
from datetime import timedelta
from osgar.node import Node


class RTKFilter(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('filtered')
        self.verbose = False
        self.trigger_time = None  # not defined
        self.buf = b''

    def on_nmea(self, data):
        if self.trigger_time is None:
            self.trigger_time = self.time + timedelta(seconds=1)
            self.buf = b''
            return  # skip first buffer
        if self.time < self.trigger_time:
            return  # not there yet
        self.buf += data
        if b'$GNGGA' in self.buf:
            packet = self.buf[self.buf.index(b'$GNGGA'):]
            if b'\r\n' in packet:
                self.publish('filtered', packet[:packet.index(b'\r\n')+2])
                self.trigger_time = self.time + timedelta(seconds=1)
                self.buf = b''

# vim: expandtab sw=4 ts=4
