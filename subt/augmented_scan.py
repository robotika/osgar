"""
  Augmented (Reality) scan with virtual obstacles

  The obstacles are (Time)Spheres of given 3D center (x, y, z) and radius.
  Some future version can limit their validity time for certain robots
  or typs of robots (UGV only, for example).
"""
import math

import numpy as np
from scipy.interpolate import interp1d  # quick and dirty workaround - to be removed

from osgar.node import Node
from osgar.lib import quaternion


UNLIMITED = 0xFFFF


def compute_scan360(xyz, radius, limit=10.0):
    """
    compute raw scan with 360 degrees
    - ignore yaw, higher resolution, offset and Z coordinate
    - use maxuint16 for unlimited
    """
    scan = np.zeros(shape=(360,), dtype=np.uint16)
    scan[:] = UNLIMITED
    x, y, z = xyz
    dist = math.hypot(x, y)
    if 0 < dist - radius < limit:
        # cut of sphere is always a circle
        angle = math.atan2(y, x)  # central angle
        variation = math.asin(radius/dist)
        i = int(math.degrees(angle))
        j = int(math.degrees(variation))
        if i < 0:
            i += 360
        value = int((dist - radius) * 1000)
        scan[max(0, i-j):i+j+1] = value
        if i - j < 0:
            scan[i-j:] = value
    return scan


class AugmentedScan(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register("scan", "scan360")
        self.barrier = [[10.0, 0.0, 0.0], 5.0]  # experiment, was None  # updated remotely via Node
        self.xyz, self.quat = None, None

    def on_scan(self, data):
        if self.barrier is None or self.xyz is None or self.quat is None:
            self.publish('scan', data)
        else:
            assert len(self.barrier) == 2, self.barrier  # expected 1 barrier [[x, y, z], radius]
            arr = np.array(self.barrier[0]) - np.array(self.xyz)
            tmp = compute_scan360(arr, radius=self.barrier[1])
            index = int(math.degrees(quaternion.heading(self.quat)))
            tmp = np.concatenate((tmp, tmp, tmp))[index+360-135:index+360+135]
            scan = np.array(data, dtype=np.uint16)
            mask = (scan == 0)
            scan[mask] = UNLIMITED
            scan = np.minimum(scan, tmp)
            mask = (scan == UNLIMITED)
            scan[mask] = 0
            self.publish('scan', scan.tolist())

    def on_scan360(self, data):
        if self.barrier is None or self.xyz is None or self.quat is None:
            self.publish('scan', data)
        else:
            assert len(data) == 692, len(data)  # a bit strange resolution for the drone?!
            assert len(self.barrier) == 2, self.barrier  # expected 1 barrier [[x, y, z], radius]
            arr = np.array(self.barrier[0]) - np.array(self.xyz)
            tmp = compute_scan360(arr, radius=self.barrier[1])
            index = int(math.degrees(quaternion.heading(self.quat)))
            tmp = np.concatenate((tmp, tmp, tmp))[index+360-180:index+360+180]
            scan = np.array(data, dtype=np.uint16)
            mask = (scan == 0)
            scan[mask] = UNLIMITED
            xp = np.arange(0, len(tmp), len(tmp)/len(scan))
            nearest = interp1d(np.arange(len(scan)), scan, kind='nearest')
            scan = np.minimum(scan, nearest(xp))
            mask = (scan == UNLIMITED)
            scan[mask] = 0
            self.publish('scan360', scan.tolist())

    def on_pose3d(self, data):
        self.xyz, self.quat = data

    def on_barrier(self, data):
        pass

    def update(self):
        channel = super().update()
        handler = getattr(self, "on_" + channel, None)
        if handler is not None:
            handler(getattr(self, channel))
        else:
            assert False, channel  # not supported

# vim: expandtab sw=4 ts=4
