"""
  Convert depth images to B&W where White is danger
"""

import cv2
import numpy as np


class DepthParams:
    def __init__(
            self,
            # Focal length.
            fx=462.1,
            # Placement of the camera reliative to the center of the robot.
            camera_xyz=(0.23, 0, (0.19 + 0.06256005)),
            # Image dimensions.
            image_size=(640, 360),
            # Principal point, position of optical axis
            principal_point=(640 / 2. +  0.5, 360 / 2 + 0.5),
            max_xy=(16., 16.),
            # Low-height stuff on the ground does not matter. (In meters.)
            min_z=0.08,
            # Stuff above the robot does not matter.
            max_z=1.5,
            # We compare pixels this far away from each other vertically.
            # (In pixels.)
            vertical_pixel_offset=1,
            # How close to the vertical does an obstacle need to be?
            # (In radians.)
            vertical_diff_limit=np.radians(60),
            # How much of a slope do we still consider "normal".
            max_slope=np.radians(-16),
            # Despeckling parameters.
            noise_filter_window=(5, 5),
            noise_filter_threshold=20):

        self.fx = fx
        self.camera_xyz = np.asarray(camera_xyz)
        self.camw, self.camh = image_size
        self.cam_low = self.camh // 2 + 1
        self.rx, self.ry = principal_point
        self.max_x, self.max_y = max_xy
        self.min_z = min_z
        self.max_z = max_z
        self.vertical_pixel_offset = vertical_pixel_offset
        self.vertical_diff_limit = vertical_diff_limit
        self.max_slope = max_slope
        self.noise_filter_window = noise_filter_window
        self.noise_filter_threshold = noise_filter_threshold

        # Pixel coordinates relative to the center of the image, with positive
        # directions to the left and up.
        pxs = self.rx - np.repeat(
                np.arange(self.camw).reshape((1, self.camw)), self.camh, axis=0)
        pys = self.ry - np.repeat(
                np.arange(self.camh).reshape((self.camh, 1)), self.camw, axis=1)
        pzs = np.ones((self.camh, self.camw), dtype=np.float)
        # For each pixel in the image, a vector representing its corresponding
        # direction in the scene with a unit forward axis.
        self.ps = np.dstack([pzs, pxs / fx, pys / fx])


# Indices of directions in a matrix with 3D points.
X, Y, Z = 0, 1, 2


def depth2danger(depth_mm, params=DepthParams()):
    # COPY & PASTE - refactoring needed!

    depth = depth_mm * 0.001  # Converting to meters.
    # 3D coordinates of points detected by the depth camera, converted to
    # robot's coordinate system.
    xyz = (params.ps * np.expand_dims(depth, axis=Z) +
           [[[params.camx, param.camy, param.camz]]])
    # Relative positions of 3D points placed params.vertical_pixel_offset pixels
    # above each other.
    rel_xyz = (xyz[:-params.vertical_pixel_offset,:,:] -
               xyz[params.vertical_pixel_offset:,:,:])
    # Slope of a line connecting two points in the vertical plane they
    # define.
    slope = np.arctan2(
            rel_xyz[:,:,Z], np.hypot(rel_xyz[:,:,X], rel_xyz[:,:,Y]))

    danger = np.logical_and.reduce(np.stack([
        # It has to be near.
        np.minimum(xyz[:-params.vertical_pixel_offset,:,X],
                   xyz[params.vertical_pixel_offset:,:,X]) <= params.max_x,
        np.abs(
            np.minimum(xyz[:-params.vertical_pixel_offset,:,Y],
                       xyz[params.vertical_pixel_offset:,:,Y])) <= params.max_y,
        # It should not be something small on the ground.
        np.maximum(xyz[:-params.vertical_pixel_offset,:,Z],
                   xyz[params.vertical_pixel_offset:,:,Z]) >= params.min_z,
        # It has to be close to vertical.
        np.abs(slope - np.radians(90)) <= params.vertical_diff_limit,
        # It should not be too high above ground.
        np.maximum(xyz[:-params.vertical_pixel_offset, :, Z],
                   xyz[params.vertical_pixel_offset:, :, Z]) <= params.max_z]))

    # Filter out noise.
    danger = cv2.filter2D(
            danger.astype(np.float),
            -1,
            np.ones(params.noise_filter_window)) > params.noise_filter_threshold

    return danger


def depth2dist(depth_mm, pitch=None, roll=None, params=DepthParams()):
    # return line in mm corresponding to scan
    # optional pitch and roll angles are in radians
    # warning: there is a bug (?) in either Osgar or in SubT and the angles
    # are as if the robot was upside down.
    pitch_rot = np.asmatrix(np.eye(3))
    roll_rot = np.asmatrix(np.eye(3))
    # http://planning.cs.uiuc.edu/node102.html
    if pitch is not None:
        inv_pitch = -pitch # Pitch corrected for the upside down bug.
        pitch_rot = np.matrix(
                [[np.cos(inv_pitch), 0.0, np.sin(inv_pitch)],
                 [0.0, 1.0, 0.0],
                 [-np.sin(inv_pitch), 0.0, np.cos(inv_pitch)]])
    if roll is not None:
        inv_roll = -(roll + np.pi) # Roll corrected for the upside down bug.
        roll_rot = np.matrix(
                [[1.0, 0.0, 0.0],
                 [0.0, np.cos(inv_roll), -np.sin(inv_roll)],
                 [0.0, np.sin(inv_roll), np.cos(inv_roll)]])
    rot = roll_rot * pitch_rot

    depth = depth_mm * 0.001  # Converting to meters.
    # 3D coordinates of points detected by the depth camera, converted to
    # robot's coordinate system.
    rxyz = (params.ps * np.expand_dims(depth, axis=Z) +
            params.camera_xyz.reshape((1, 1, 3)))
    # Compensate for robot's pitch & roll.
    gxyz = np.asarray(
        (rot * np.asmatrix(rxyz.reshape((-1, 3)).T)).T).reshape(
                (params.camh, params.camw, 3))
    # Relative positions of 3D points placed params.vertical_pixel_offset pixels
    # above each other.
    rel_xyz = (gxyz[:-params.vertical_pixel_offset,:,:] -
               gxyz[params.vertical_pixel_offset:,:,:])
    # Slope of a line connecting two points in the vertical plane they
    # define.
    slope = np.arctan2(
            rel_xyz[:,:,Z], np.hypot(rel_xyz[:,:,X], rel_xyz[:,:,Y]))

    danger = np.logical_and.reduce(np.stack([
        # It has to be near.
        np.minimum(rxyz[:-params.vertical_pixel_offset,:,X],
                   rxyz[params.vertical_pixel_offset:,:,X]) <= params.max_x,
        np.abs(
            np.minimum(
                rxyz[:-params.vertical_pixel_offset,:,Y],
                rxyz[params.vertical_pixel_offset:,:,Y])) <= params.max_y,
        # It should not be something small on the ground.
        np.maximum(rxyz[:-params.vertical_pixel_offset,:,Z],
                   rxyz[params.vertical_pixel_offset:,:,Z]) >= params.min_z,
        # It has to be close to vertical.
        np.abs(slope - np.radians(90)) <= params.vertical_diff_limit,
        # It should not be too high above ground.
        np.maximum(rxyz[:-params.vertical_pixel_offset, :, Z],
                   rxyz[params.vertical_pixel_offset:, :, Z]) <= params.max_z]))

    # Filter out noise.
    danger = cv2.filter2D(
            danger.astype(np.float),
            -1,
            np.ones(params.noise_filter_window)) > params.noise_filter_threshold

    dists = np.hypot(rxyz[:,:,X], rxyz[:,:,Y])
    FAR_AWAY = 1000.0
    dists = np.where(danger, dists[params.vertical_pixel_offset:,:], FAR_AWAY)
    scan = np.min(dists, axis=0)
    mask = scan == FAR_AWAY
    scan[mask] = 0

    # down drops
    low_rxyz = rxyz[params.cam_low:,:]
    low_gxyz = gxyz[params.cam_low:,:]
    gdists = np.hypot(low_gxyz[:,:,X], low_gxyz[:,:,Y])
    slopes = np.arctan2(low_gxyz[:,:,Z], gdists)
    below_ground = slopes < params.max_slope

    # Let's calculate, where we should see ground.
    q = params.camera_xyz[Z] / np.maximum(
            1e-6, (params.camera_xyz[Z] - low_rxyz[:,:,Z]))
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
    import json
    import matplotlib.pyplot as plt

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('filename', help='NPZ file with depth data')
    parser.add_argument('--depth-params', type=json.loads,
                        help='Depth processing parameters in json format. ' +
                              'See DepthParams.')
    args = parser.parse_args()

    with np.load(args.filename) as f:
        depth = f['depth']

    depth_params = DepthParams(**args.depth_params)
    dist = depth2dist(depth, params=depth_params)

    plt.plot(range(640), dist, 'o-', linewidth=2)
    plt.show()

# vim: expandtab sw=4 ts=4

