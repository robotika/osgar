"""
  Convert depth image to "lidar" scan
"""
import math

import numpy as np

from osgar.node import Node
from osgar.bus import BusShutdownException
from osgar.lib.depth import depth2danger, danger2dist


frac = math.tan(math.radians(60))/360


def vertical_scan(depth, column):
    # return two arrays x and y
    arr = np.array(depth[:, column], np.int32)
    x = arr
    a = frac * (np.arange(len(arr), 0, -1) - 180)
    y = x * a + 560
    return x, y


def monotize(arr):
    """change function to monotonous increasing"""
    best = arr[0]
    for i in range(len(arr)):  # TODO optimize
        best = max(best, arr[i])
        arr[i] = best
    return arr


def find_step(arr):
    """input is uniform array of values"""
    INDEX_STEP = 10  # so for 10cm step there is 60mm height difference -> ~30deg
    THRESHOLD = 60
    diff = arr[INDEX_STEP:] - arr[:-INDEX_STEP]
    mask = diff > THRESHOLD
    index = np.argmax(mask)
    if mask[index]:
        return index + INDEX_STEP
    return None


def vertical_step(depth, column=320):
    """Detect nearest obstacle for vertical line"""
    MAX_SIZE = 250  # in cm, 1cm resolution
    x_arr, y_arr = vertical_scan(depth, column)
    x_arr //= 10  # reduction to cm
    d = np.zeros(MAX_SIZE, np.int32)
    for x, y in zip(x_arr, y_arr):
        if 0 <= x < MAX_SIZE:
            d[x] = max(d[x], y)
    ret = find_step(monotize(d))
    if ret is None:
        return None
    return ret * 10  # return back readings in mm


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
            dist = self.depth[danger2dist(depth2danger(self.depth)), np.arange(640)]

            # 60*720/270 = 160.0 ... i.e. 160 elements to be replaced
            # 640/160 = 4.0 ... i.e. downsample by 4
            small = np.array(dist[::4], dtype=np.int32)  # problem with abs() of uint16
            mask = small == 0xFFFF
            small[mask] = 0            
            new_scan = self.scan[:720//2-80] + small.tolist() + self.scan[720//2+80:]
            assert len(new_scan) == 720, len(new_scan)
            self.publish('scan', new_scan)
        else:
            assert False, channel  # unsupported channel

        return channel

if __name__ == '__main__':
    import argparse
    import matplotlib.pyplot as plt

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('filename', help='NPZ file with depth data')
    parser.add_argument('-v', '--verbose', help='verbose mode', action='store_true')
    parser.add_argument('-d', '--draw', help='draw graph(s)', action='store_true')
    args = parser.parse_args()

    with np.load(args.filename) as f:
        depth = f['depth']

    if args.draw:
        for i in range(0, 640, 159):
            x, y = vertical_scan(depth, i)
            plt.plot(x, y, 'o-', linewidth=2)

            d =  vertical_step(depth, i)
            plt.plot([d, d], [0, 200], '-', linewidth=3)

        plt.axes().set_aspect('equal', 'datalim')
        plt.show()

# vim: expandtab sw=4 ts=4
