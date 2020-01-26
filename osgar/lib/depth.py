"""
  Convert depth images to B&W where White is danger
"""

import cv2
import numpy as np
import sys

FX = 462.1  # Focal length.
# Placement of the camera with rspect to the center of the robot.
CAMX, CAMY, CAMZ = 0.23, 0, (0.19 + 0.06256005)
# Image dimensions.
CAMW, CAMH = 640, 360

# How far we care. (In meters.)
MAXX = 5.
MAXY = 5.

# Low-height stuff on the ground does not matter. (In meters.)
MINZ = 0.06
MAXZ = 1.5

# We compare pixels this far away from each other vertically. (In pixels.)
OFFSET = 1

# How close to the vertical does an obstacle need to be? (In radians.)
VERTICAL_DIFF_LIMIT = np.radians(60)


# Which rows of the image do we consider "low".
CAM_LOW = CAMH // 2 + 1

# How much of a slope do we still consider "normal".
MAX_SLOPE = np.radians(-15)


# Indices of directions in a matrix with 3D points.
X, Y, Z = 0, 1, 2

# Pixel coordinates relative to the center of the image, with positive
# directions to the left and up.
pxs = CAMW / 2. - np.repeat(np.arange(CAMW).reshape((1, CAMW)), CAMH, axis=0)
pys = CAMH / 2. - np.repeat(np.arange(CAMH).reshape((CAMH, 1)), CAMW, axis=1)
pzs = np.ones((CAMH, CAMW), dtype=np.float)
# For each pixel in the image, a vector representing its corresponding direction
# in the scene with a unit forward axis.
ps = np.dstack([pzs, pxs / FX, pys / FX])


def depth2danger(depth_mm):
    # COPY & PASTE - refactoring needed!

    depth = depth_mm * 0.001  # Converting to meters.
    # 3D coordinates of points detected by the depth camera, converted to
    # robot's coordinate system.
    xyz = ps * np.expand_dims(depth, axis=Z) + [[[CAMX, CAMY, CAMZ]]]
    # Relative positions of 3D points placed OFFSET pixels above each other.
    rel_xyz = xyz[:-OFFSET,:,:] - xyz[OFFSET:,:,:]
    # Slope of a line connecting two points in the vertical plane they
    # define.
    slope = np.arctan2(
            rel_xyz[:,:,Z], np.hypot(rel_xyz[:,:,X], rel_xyz[:,:,Y]))

    danger = np.logical_and.reduce(np.stack([
        # It has to be near.
        np.minimum(xyz[:-OFFSET,:,X], xyz[OFFSET:,:,X]) <= MAXX,
        np.minimum(xyz[:-OFFSET,:,Y], xyz[OFFSET:,:,Y]) <= MAXY,
        # It should not be something small on the ground.
        np.maximum(xyz[:-OFFSET,:,Z], xyz[OFFSET:,:,Z]) >= MINZ,
        # It has to be close to vertical.
        np.abs(slope - np.radians(90)) <= VERTICAL_DIFF_LIMIT,
        # It should not be too high above ground.
        np.maximum(xyz[:-OFFSET, :, Z], xyz[OFFSET:, :, Z]) <= MAXZ]))

    # Filter out noise.
    danger = cv2.filter2D(danger.astype(np.float), -1, np.ones((3, 3))) > 7

    return danger


def depth2dist(depth_mm):
    # return line in mm corresponding to scan

    depth = depth_mm * 0.001  # Converting to meters.
    # 3D coordinates of points detected by the depth camera, converted to
    # robot's coordinate system.
    xyz = ps * np.expand_dims(depth, axis=Z) + [[[CAMX, CAMY, CAMZ]]]
    # Relative positions of 3D points placed OFFSET pixels above each other.
    rel_xyz = xyz[:-OFFSET,:,:] - xyz[OFFSET:,:,:]
    # Slope of a line connecting two points in the vertical plane they
    # define.
    slope = np.arctan2(
            rel_xyz[:,:,Z], np.hypot(rel_xyz[:,:,X], rel_xyz[:,:,Y]))

    danger = np.logical_and.reduce(np.stack([
        # It has to be near.
        np.minimum(xyz[:-OFFSET,:,X], xyz[OFFSET:,:,X]) <= MAXX,
        np.minimum(xyz[:-OFFSET,:,Y], xyz[OFFSET:,:,Y]) <= MAXY,
        # It should not be something small on the ground.
        np.maximum(xyz[:-OFFSET,:,Z], xyz[OFFSET:,:,Z]) >= MINZ,
        # It has to be close to vertical.
        np.abs(slope - np.radians(90)) <= VERTICAL_DIFF_LIMIT,
        # It should not be too high above ground.
        np.maximum(xyz[:-OFFSET, :, Z], xyz[OFFSET:, :, Z]) <= MAXZ]))

    # Filter out noise.
    danger = cv2.filter2D(danger.astype(np.float), -1, np.ones((3, 3))) > 7

    dists = np.hypot(xyz[:,:,X], xyz[:,:,Y])
    FAR_AWAY = 1000.0
    dists = np.where(danger, dists[OFFSET:,:], FAR_AWAY)
    scan = np.min(dists, axis=0)
    mask = scan == FAR_AWAY
    scan[mask] = 0

    # down drops
    low_xyz = ps[CAM_LOW:,:] * np.expand_dims(
            depth[CAM_LOW:,:], axis=Z) + [[[CAMX, CAMY, CAMZ]]]
    dists = np.hypot(low_xyz[:,:,X], low_xyz[:,:,Y])
    slopes = np.arctan2(low_xyz[:,:,Z], dists)
    below_ground = slopes < MAX_SLOPE

    # Let's calculate, where we should see ground.
    q = CAMZ / np.maximum(1e-6, (CAMZ - low_xyz[:,:,Z]))
    expected_ground = low_xyz * np.expand_dims(q, 2)
    ground_dists = np.hypot(expected_ground[:,:,X], expected_ground[:,:,Y])
    FAR_AWAY = 1000.
    scan_down = np.min(
            np.where(below_ground, ground_dists, FAR_AWAY),
            axis=0)

    mask = scan_down < FAR_AWAY
    scan[mask] = scan_down[mask]  # down drop readings are more important then walls

    scan = np.array(scan*1000, dtype=np.int32)
    return scan


if __name__ == '__main__':
    import argparse
    import matplotlib.pyplot as plt

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('filename', help='NPZ file with depth data')
    args = parser.parse_args()

    with np.load(args.filename) as f:
        depth = f['depth']

    dist = depth2dist(depth)

    plt.plot(range(640), dist, 'o-', linewidth=2)
    plt.show()

# vim: expandtab sw=4 ts=4

