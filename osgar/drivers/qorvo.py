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

    def update(self):  # yes, refactoring to some common node would be nice!
        channel = super().update()  # define self.time
        handler = getattr(self, "on_" + channel, None)
        if handler is not None:
            handler(getattr(self, channel))
        else:
            assert False, channel  # unknown


# vim: expandtab sw=4 ts=4
