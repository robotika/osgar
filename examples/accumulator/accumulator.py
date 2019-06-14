"""
  Example of a simple application collecting map data
"""
from osgar.node import Node
from osgar.bus import BusShutdownException


class Processor(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)

    def update(self):
        channel = super().update()
        assert channel == 'map', channel
        print(self.time, len(self.map))
        self.sleep(0.2)  # simulate some work here
        self.publish('request', True)

    def run(self):
        try:
            self.publish('request', True)
            while True:
                self.update()
        except BusShutdownException:
            pass


class Accumulator(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        self.map_data = []

    def update(self):
        channel = super().update()
        if channel == 'request':
            self.publish('map', self.map_data)
        elif channel == 'xyz':
            self.map_data.extend(self.xyz)
            self.map_data = self.map_data[-100:]  # keep last 100 points

# vim: expandtab sw=4 ts=4
