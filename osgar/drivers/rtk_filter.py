"""
  RTK filter (reduce GPS NMEA to 1Hz for query to server)
"""
from osgar.node import Node


class RTKFilter(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('filtered')
        self.verbose = False

    def on_nmea(self, data):
        self.publish('filtered', data)


# vim: expandtab sw=4 ts=4
