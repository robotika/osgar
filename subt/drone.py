"""
  OSGAR Drone for control of flight height
"""
import math

from osgar.node import Node


class Drone(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register("desired_speed_3d")

    def on_desired_speed(self, data):
        linear = [data[0]/1000, 0.0, 0.0]
        angular = [0.0, 0.0, math.radians(data[1]/100.0)]
        self.publish('desired_speed_3d', [linear, angular])

    def update(self):
        channel = super().update()

        handler = getattr(self, "on_" + channel, None)
        if handler is not None:
            handler(getattr(self, channel))

# vim: expandtab sw=4 ts=4
