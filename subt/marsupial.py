"""
  Dual robots - typically a UGV taking care of UAV
  https://en.wikipedia.org/wiki/Marsupial
"""
from ast import literal_eval

from osgar.node import Node
from subt.trace import distance3D


class Marsupial(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register("detach")

        self.start_time = None  # unknown
        self.release_at = config.get('release_at')
        self.drone_available = True

    def detach(self):
        self.publish('deploy', [])  # ROS std_msg/Empty expects empty list as input
        self.drone_available = False

    def on_sim_time_sec(self, data):
        if self.start_time is None:
            self.start_time = data

        if self.release_at is None:
            return

        if self.start_time + self.release_at <= data and self.drone_available:
            self.detach()

    def update(self):
        channel = super().update()
        handler = getattr(self, "on_" + channel, None)
        if handler is not None:
            handler(getattr(self, channel))

# vim: expandtab sw=4 ts=4
