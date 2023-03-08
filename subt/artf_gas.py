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
        bus.register("localized_artf")
        self.xyz = None
        self.last_bottom_scan = 0.0  # expected to never arrive for ground robots

    def on_gas_detected(self, data):
        if data and self.xyz is not None:
            x, y, z = self.xyz
            self.publish('localized_artf', [GAS, [x, y, z - self.last_bottom_scan]])

    def on_pose3d(self, data):
        self.xyz = data[0]  # ignore orientation

    def on_bottom_scan(self, data):
        self.last_bottom_scan = data[0]

# vim: expandtab sw=4 ts=4
