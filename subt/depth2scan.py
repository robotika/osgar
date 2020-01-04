"""
  Convert depth image to "lidar" scan
"""
import numpy as np

from osgar.node import Node
from osgar.bus import BusShutdownException


class DepthToScan(Node):
    deg30 = int(30*720/270)

    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register("scan")
        self.depth = None  # initialize inputs
        self.scan = None
        self.verbose = False

    def update(self):
        channel = super().update()
        assert channel in ["depth", "scan"], channel

        if channel == 'scan':
            assert len(self.scan) == 720, len(self.scan)
        elif channel == 'depthXXX':
            assert len(self.depth) == 640 * 360, len(self.depth)
            i = (360//2)*640
            mid_line = np.array(self.depth[i:i+640], dtype=np.dtype('H'))
            # 60*720/270 = 160.0 ... i.e. 160 elements to be replaced
            # 640/160 = 4.0 ... i.e. downsample by 4
            small = mid_line[::4]
            mask = small == 0xFFFF
            small[mask] = 0
            if self.scan is not None:
                new_scan = self.scan[:360-80] + np.flip(small, axis=0).tolist() + self.scan[360+80:]
                assert len(new_scan) == 720, len(new_scan)
                self.publish('scan', new_scan)
        elif channel == 'depth':
            assert len(self.depth) == 640 * 360, len(self.depth)
            i = 300*640  # slope down lidar
            mid_line = np.array(self.depth[i:i+640], dtype=np.dtype('H'))
            # 60*720/270 = 160.0 ... i.e. 160 elements to be replaced
            # 640/160 = 4.0 ... i.e. downsample by 4
            small = np.array(mid_line[::4], dtype=np.int32)  # problem with abs() of uint16
            mask0 = small == 0xFFFF
            diff = abs(small[:-1] - small[1:])
            mask1 = diff > 100
            if self.scan is not None:
                new_scan = self.scan[:]
                for i, val in enumerate(mask1):
                    if val:
                        if self.verbose:
                            print(i, new_scan[360+79-i], int(small[i]))
                            assert False, small
                        new_scan[360+79-i] = int(small[i])
                assert len(new_scan) == 720, len(new_scan)
                self.publish('scan', new_scan)
        else:
            assert False, channel  # unsupported channel


# vim: expandtab sw=4 ts=4
