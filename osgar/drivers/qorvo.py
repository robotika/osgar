"""
  Simple(minded) driver for Qorvo UWB modules
"""

from osgar.node import Node


class Qorvo(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('range', 'raw')
        self.verbose = False
        self.initialized = False

    def on_raw(self, data):
        if not self.initialized:
            self.publish("raw", b'la\n')
        self.initialized = True

    def on_timer(self, data):
        if not self.initialized:
            self.publish("raw", b'\n')


# vim: expandtab sw=4 ts=4
