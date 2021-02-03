"""
  Driver for a mini drone Tello

  for details see blog at
     https://robotika.cz/robots/paula/
"""

from osgar.node import Node
from osgar.bus import BusShutdownException


class TelloDrone(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('cmd')

    def run(self):
        self.publish('cmd', b'command')
        try:
            while True:
                self.update()
        except BusShutdownException:
            pass

# vim: expandtab sw=4 ts=4
