"""
  Augmented (Reality) scan with virtual obstacles
"""
from osgar.node import Node


class AugmentedScan(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register("scan")

    def on_scan(self, data):
        self.publish('scan', data)

    def on_pose3d(self, data):
        xyz, quat = data

    def on_barrier(self, data):
        pass

    def update(self):
        channel = super().update()
        handler = getattr(self, "on_" + channel, None)
        if handler is not None:
            handler(getattr(self, channel))

# vim: expandtab sw=4 ts=4
