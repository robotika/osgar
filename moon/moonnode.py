"""
  MoonNode - parent for Moon processing nodes
"""
from datetime import timedelta

from osgar.node import Node


class MoonNode(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        self.sim_time = None
        self.monitors = []

    def on_sim_clock(self, data):
        self.sim_time = timedelta(seconds=data[0], microseconds=data[1])

    def update(self):
        channel = super().update()

        handler = getattr(self, "on_" + channel, None)
        if handler is not None:
            handler(getattr(self, channel))

        for m in self.monitors:
            m(self, channel)

        return channel

# vim: expandtab sw=4 ts=4
