"""
  OSGAR Gas detector
"""
# This is the simplest version taken from subt/artifacts.py - i.e. use reference position as
# the first moment when the robot detects gas. More advanced/precise implementations can take the map
# or scan and report the entering doors as defined in the rules.

from osgar.node import Node
from subt.artifacts import GAS


class ArtifactGasDetector(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register("artf")

    def on_gas_detected(self, data):
        # [type, [rel_x, rel_y, rel_z]]
        if data:
            self.publish('artf', [GAS, [0, 0, 0]])

    def update(self):
        channel = super().update()
        handler = getattr(self, "on_" + channel, None)
        if handler is not None:
            handler(getattr(self, channel))
        else:
            assert False, channel  # not supported channel

# vim: expandtab sw=4 ts=4
