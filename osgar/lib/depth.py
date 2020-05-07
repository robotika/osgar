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
MAXX = 16.
MAXY = 16.

# Low-height stuff on the ground does not matter. (In meters.)
MINZ = 0.08
MAXZ = 1.5

# We compare pixels this far away from each other vertically. (In pixels.)
OFFSET = 1

# How close to the vertical does an obstacle need to be? (In radians.)
VERTICAL_DIFF_LIMIT = np.radians(60)


# Which rows of the image do we consider "low".
CAM_LOW = CAMH // 2 + 1

# How much of a slope do we still consider "normal".
MAX_SLOPE = np.radians(-16)


# Indices of directions in a matrix with 3D points.
X, Y, Z = 0, 1, 2

# Pixel coordinates relative to the center of the image, with positive
# directions to the left and up.
pxs = CAMW / 2. + 0.5 - np.repeat(np.arange(CAMW).reshape((1, CAMW)), CAMH, axis=0)
pys = CAMH / 2. + 0.5 - np.repeat(np.arange(CAMH).reshape((CAMH, 1)), CAMW, axis=1)
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
    danger = cv2.filter2D(danger.astype(np.float), -1, np.ones((5, 5))) > 20

    return danger


def depth2dist(depth_mm, pitch=None, roll=None):
    # return line in mm corresponding to scan
    # optional pitch and roll angles are in radians
    # warning: there is a bug (?) in either Osgar or in SubT and the angles
    # are as if the robot was upside down.
    pitch_rot = np.asmatrix(np.eye(3))
    roll_rot = np.asmatrix(np.eye(3))
    if pitch is not None:
        inv_pitch = -(-pitch) # Opposite angle to pitch corrected for the
                              # upside down bug.
        pitch_rot = np.matrix(
                [[np.cos(inv_pitch), 0.0, -np.sin(inv_pitch)],
                 [0.0, 1.0, 0.0],
                 [np.sin(inv_pitch), 0.0, np.cos(inv_pitch)]])
    if roll is not None:
        inv_roll = -(roll + np.pi) # Opposite angle to roll corrected for the
                                   # upside down bug.
        roll_rot = np.matrix(
                [[1.0, 0.0, 0.0],
                 [0.0, np.cos(inv_roll), -np.sin(inv_roll)],
                 [0.0, np.sin(inv_roll), np.cos(inv_roll)]])
    rot = roll_rot * pitch_rot

    depth = depth_mm * 0.001  # Converting to meters.
    # 3D coordinates of points detected by the depth camera, converted to
    # robot's coordinate system.
    rxyz = ps * np.expand_dims(depth, axis=Z) + [[[CAMX, CAMY, CAMZ]]]
    # Compensate for robot's pitch & roll.
    gxyz = np.asarray(
        (rot * np.asmatrix(rxyz.reshape((-1, 3)).T)).T).reshape((CAMH, CAMW, 3))
    # Relative positions of 3D points placed OFFSET pixels above each other.
    rel_xyz = gxyz[:-OFFSET,:,:] - gxyz[OFFSET:,:,:]
    # Slope of a line connecting two points in the vertical plane they
    # define.
    slope = np.arctan2(
            rel_xyz[:,:,Z], np.hypot(rel_xyz[:,:,X], rel_xyz[:,:,Y]))

    danger = np.logical_and.reduce(np.stack([
        # It has to be near.
        np.minimum(rxyz[:-OFFSET,:,X], rxyz[OFFSET:,:,X]) <= MAXX,
        np.minimum(rxyz[:-OFFSET,:,Y], rxyz[OFFSET:,:,Y]) <= MAXY,
        # It should not be something small on the ground.
        np.maximum(rxyz[:-OFFSET,:,Z], rxyz[OFFSET:,:,Z]) >= MINZ,
        # It has to be close to vertical.
        np.abs(slope - np.radians(90)) <= VERTICAL_DIFF_LIMIT,
        # It should not be too high above ground.
        np.maximum(rxyz[:-OFFSET, :, Z], rxyz[OFFSET:, :, Z]) <= MAXZ]))

    # Filter out noise.
    danger = cv2.filter2D(danger.astype(np.float), -1, np.ones((5, 5))) > 20

    dists = np.hypot(rxyz[:,:,X], rxyz[:,:,Y])
    FAR_AWAY = 1000.0
    dists = np.where(danger, dists[OFFSET:,:], FAR_AWAY)
    scan = np.min(dists, axis=0)
    mask = scan == FAR_AWAY
    scan[mask] = 0

    # down drops
    low_rxyz = rxyz[CAM_LOW:,:]
    low_gxyz = gxyz[CAM_LOW:,:]
    gdists = np.hypot(low_gxyz[:,:,X], low_gxyz[:,:,Y])
    slopes = np.arctan2(low_gxyz[:,:,Z], gdists)
    below_ground = slopes < MAX_SLOPE

    # Let's calculate, where we should see ground.
    q = CAMZ / np.maximum(1e-6, (CAMZ - low_rxyz[:,:,Z]))
    expected_ground = low_rxyz * np.expand_dims(q, 2)
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

