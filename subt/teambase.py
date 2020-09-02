"""
  OSGAR Teambase for Virtual
"""
from osgar.node import Node


class Teambase(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register("broadcast")

    def on_sim_time_sec(self, data):
        # broadcast simulation time every second
        self.publish('broadcast', b'%d' % data)

    def update(self):
        channel = super().update()

        handler = getattr(self, "on_" + channel, None)
        if handler is not None:
            handler(getattr(self, channel))

# vim: expandtab sw=4 ts=4
