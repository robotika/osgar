"""
  Example of a simple application collecting map data
"""
from osgar.node import Node


class Processor(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)

    def update(self):
        channel = super().update()


class Accumulator(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)

    def update(self):
        channel = super().update()

# vim: expandtab sw=4 ts=4
