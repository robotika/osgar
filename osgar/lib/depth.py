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
MINZ = 0.08

# We compare pixels this far away from each other vertically. (In pixels.)
OFFSET = 1

# How close to the vertical does an obstacle need to be? (In radians.)
VERTICAL_DIFF_LIMIT = np.radians(45)

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
        np.abs(slope - np.radians(90)) <= VERTICAL_DIFF_LIMIT]))
    return danger


def danger2dist(danger):
    # we need closes obstacles to robot
    # argmax of last element
    # https://stackoverflow.com/questions/8768540/how-to-find-last-occurrence-of-maximum-value-in-a-numpy-ndarray/8768734
    rev = danger[::-1]
    d = np.argmax(rev, axis=0)
    d = 359 - d
    return d

# vim: expandtab sw=4 ts=4

