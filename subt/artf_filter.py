"""
  Filter of detector outputs to unique reports
"""

from osgar.node import Node
from osgar.bus import BusShutdownException


class ArtifactFilter(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register("localized_artf")



# vim: expandtab sw=4 ts=4
