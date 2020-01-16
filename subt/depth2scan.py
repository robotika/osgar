"""
  Convert depth image to "lidar" scan
"""
import numpy as np

from osgar.node import Node
from osgar.bus import BusShutdownException


def vertical_step(depth, verbose=False):
    """Detect nearest obstacle for vertical line"""
    d = np.array(depth[:, 320], np.int32)
    if verbose:
        print(d)


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

        if channel == 'depth':
            assert self.depth.shape == (360, 640), self.depth.shape
        elif channel == 'scan':
            assert len(self.scan) == 720, len(self.scan)
            if self.depth is None:
                self.publish('scan', self.scan)
                return channel  # when no depth data are available ...
            i = 300  # slope down lidar ... used to be (360//2)
            mid_line = self.depth[i]
            # 60*720/270 = 160.0 ... i.e. 160 elements to be replaced
            # 640/160 = 4.0 ... i.e. downsample by 4
            small = np.array(mid_line[::4], dtype=np.int32)  # problem with abs() of uint16
            mask0 = small == 0xFFFF
            diff = abs(small[:-1] - small[1:])
            mask1 = diff > 100
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

        return channel

if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('filename', help='NPZ file with depth data')
    parser.add_argument('-v', '--verbose', help='verbose mode', action='store_true')
    args = parser.parse_args()

    with np.load(args.filename) as f:
        depth = f['depth']

    vertical_step(depth, verbose=args.verbose)

# vim: expandtab sw=4 ts=4
