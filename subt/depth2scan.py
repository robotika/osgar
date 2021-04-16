"""
  Convert depth image to "lidar" scan
"""
import math

import cv2
import numpy as np

from osgar.node import Node
from osgar.bus import BusShutdownException
from osgar.lib.depth import depth2dist, DepthParams, decompress as decompress_depth
from osgar.lib.quaternion import multiply as multiply_quaternions, euler_zyx


FRAC = math.tan(math.radians(60)) / 360


def vertical_scan(depth, column):
    # return two arrays x and y
    arr = np.array(depth[:, column] * 1000, np.int32)
    x = arr
    a = FRAC * (np.arange(len(arr), 0, -1) - 180)
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


def is_on_line(a, b, c, z=300):
    "Is point C at most `z` milimeters far from a line defined by points A and B?"
    ax, ay = a
    bx, by = b
    cx, cy = c
    p = -(by - ay)
    q = bx - ax
    r = -(p * ax + q * ay)
    d = (p * cx + q * cy + r) / np.hypot(p, q)
    return abs(d) < z


def adjust_scan(scan, depth_scan, depth_params):

    # Lidar scan starts with right-most direction. Let's make the
    # virtual scan based on depth camera do the same.
    depth_scan = np.asarray(depth_scan[::-1])

    lidar_scan = np.asarray(scan)

    camera_fov = 2 * np.arctan2(depth_params.camw / 2.0, depth_params.fx)
    lidar_fov = depth_params.lidar_fov

    depth_density = depth_scan.shape[0] / camera_fov
    lidar_density = lidar_scan.shape[0] / lidar_fov
    assert depth_density > lidar_density
    depth_scan = cv2.resize(
        depth_scan.reshape((1, -1)),
        (int(camera_fov * lidar_density), 1),
        interpolation=cv2.INTER_NEAREST).reshape((-1,))
    alignment_start = (lidar_scan.shape[0] - depth_scan.shape[0]) // 2
    alignment_tail = lidar_scan.shape[0] - depth_scan.shape[0] - alignment_start

    # Part of the lidar scan that overlaps with depth camera.
    lidar_overlap = lidar_scan[alignment_start:-alignment_tail]

    DISAGREEMENT_MM_LIMIT = 600
    LIDAR_NO_MEASUREMENT = 0
    DEPTH_NO_MEASUREMENT = 0
    def disagreement(idx):
        return lidar_overlap[idx] != LIDAR_NO_MEASUREMENT and (
                depth_scan[idx] == DEPTH_NO_MEASUREMENT or
                depth_scan[idx] - lidar_overlap[idx] > DISAGREEMENT_MM_LIMIT)
    # We check multiple consecutive directions to reduce noise.
    disagreement_right = disagreement(0) and disagreement(1) and disagreement(2)
    disagreement_left = disagreement(-1) and disagreement(-2) and disagreement(-3)

    if disagreement_right:
        # Taking measurements 2 indices away to lower impact of noise.
        d0, d1 = lidar_scan[alignment_start], lidar_scan[alignment_start + 2]
        phi0 = -lidar_fov / 2 + alignment_start / lidar_density
        phi1 = phi0 + 2 / lidar_density
        a = d0 * np.cos(phi0), d0 * np.sin(phi0)
        b = d1 * np.cos(phi1), d1 * np.sin(phi1)
        for i in range(alignment_start):
            d2 = lidar_scan[i]
            if d2 == LIDAR_NO_MEASUREMENT or d2 < depth_params.lidar_trusted_zone:
                continue
            phi2 = -lidar_fov / 2 + i / lidar_density
            c = d2 * np.cos(phi2), d2 * np.sin(phi2)
            if is_on_line(a, b, c):
                lidar_scan[i] = LIDAR_NO_MEASUREMENT
    if disagreement_left:
        # Taking measurements 2 indices away to lower impact of noise.
        aidx = -alignment_tail - 1
        bidx = -alignment_tail - 1 - 2
        d0, d1 = lidar_scan[aidx], lidar_scan[bidx]
        phi0 = lidar_fov / 2 + aidx / lidar_density
        phi1 = phi0 - 2 / lidar_density
        a = d0 * np.cos(phi0), d0 * np.sin(phi0)
        b = d1 * np.cos(phi1), d1 * np.sin(phi1)
        lidar_len = lidar_scan.shape[0]
        for i in range(lidar_len - alignment_tail, lidar_len):
            d2 = lidar_scan[i]
            if d2 == LIDAR_NO_MEASUREMENT or d2 < depth_params.lidar_trusted_zone:
                continue
            phi2 = -lidar_fov / 2 + i / lidar_density
            c = d2 * np.cos(phi2), d2 * np.sin(phi2)
            if is_on_line(a, b, c):
                lidar_scan[i] = LIDAR_NO_MEASUREMENT

    depth_scan[depth_scan == DEPTH_NO_MEASUREMENT] = LIDAR_NO_MEASUREMENT
    new_scan = lidar_scan
    # Up to depth_params.lidar_trusted_zone, we trust the lidar measurements,
    # so we take a minimum between lidar- and depth-based distance to obstacle.
    # Beyond that distance, we no longer trust lidar, because it could, for
    # example, be pointing to ground, and we only take the depth-based estimate.
    new_scan[alignment_start:-alignment_tail] = np.where(
            np.logical_and(
                lidar_overlap != LIDAR_NO_MEASUREMENT,
                lidar_overlap < depth_params.lidar_trusted_zone),
            np.minimum(lidar_overlap,
                   # If we have a trusted lidar measurement, we shouldn't
                   # override it with "no depth measurement", even if that is
                   # technically a lower value.
                   np.where(
                       depth_scan == DEPTH_NO_MEASUREMENT,
                       np.inf,
                       depth_scan)),
            depth_scan)
    return new_scan

class DepthToScan(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register("scan")
        self.rgbd = None  # initialize inputs
        self.scan = None
        self.verbose = False
        self.depth_params = DepthParams(**config.get('depth_params', {}))
        self.filter_fog = config.get('filter_fog', False)

    def update(self):
        channel = super().update()
        assert channel in ["scan", "rgbd"], channel

        if channel == 'scan':
            depth = None
            if self.rgbd is not None:
                robot_pose, camera_pose, __rgb_compressed, depth_compressed = self.rgbd
                __robot_xyz, robot_rotation = robot_pose
                __camera_xyz, camera_rotation = camera_pose
                full_rotation = multiply_quaternions(robot_rotation, camera_rotation)
                yaw, pitch, roll = euler_zyx(full_rotation)
                depth = decompress_depth(depth_compressed)

            if depth is None:
                self.publish('scan', self.scan)
                return channel  # when no depth data are available ...

            if self.filter_fog:
                depth = cv2.medianBlur(depth, 5)
                scan = cv2.medianBlur(np.asarray(self.scan, dtype=np.uint16), 5)[:,0]
            else:
                scan = self.scan

            depth_scan = depth2dist(depth, self.depth_params, pitch, roll, yaw)
            new_scan = adjust_scan(scan, depth_scan, self.depth_params)
            self.publish('scan', new_scan.tolist())

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
