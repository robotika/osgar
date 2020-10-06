"""
  Augmented (Reality) scan with virtual obstacles
"""
import math

import numpy as np

from osgar.node import Node


def compute_scan360(barrier, limit=10.0):
    """
    compute raw scan with 360 degrees
    - ignore yaw, higher resolution, offset and Z coordinate
    """
    scan = np.zeros(shape=(360,), dtype=np.uint16)
    for x, y, z in barrier:
        dist = math.hypot(x, y)
        if dist < limit:
            angle = math.atan2(y, x)
            i = int(math.degrees(angle))
            if i < 0:
                i += 360
            scan[i] = int(dist * 1000)
    return scan


class AugmentedScan(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register("scan")
        self.barrier = []  # updated remotely via Node
        self.xyz, self.quat = None, None

    def on_scan(self, data):
        if len(self.barrier) == 0 or self.xyz is None or self.quat is None:
            self.publish('scan', data)
        else:
            arr = np.array(self.barrier) - np.array(self.xyz)
            print(arr)
            tmp = compute_scan360(arr)[:270]
            mask = tmp != 0
            scan = np.array(data)
            print(mask)
            scan[mask] = tmp
            self.publish('scan', scan.tolist())

    def on_pose3d(self, data):
        self.xyz, self.quat = data

    def on_barrier(self, data):
        pass

    def update(self):
        channel = super().update()
        handler = getattr(self, "on_" + channel, None)
        if handler is not None:
            handler(getattr(self, channel))

# vim: expandtab sw=4 ts=4
