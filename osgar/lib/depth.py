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
            # Placement of the camera relative to the center of the robot.
            camera_xyz=(0.23, 0, (0.19 + 0.06256005)),
            # Orientation of the camera relative to the robot. In radians.
            camera_ypr=(0, 0, 0),
            # Image dimensions.
            image_size=(640, 360),
            # Principal point, position of optical axis
            principal_point=(640 / 2. +  0.5, 360 / 2 + 0.5),
            min_x = 0.,
            min_y = 0.,
            max_x = 16.,
            max_y = 16.,
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
            slope_length_scale_factor = 1.0,
            slope_length_power_factor = 0.0,
            # Despeckling parameters.
            noise_filter_window=(5, 5),
            noise_filter_threshold=20,
            # Lidar field of view in radians.
            lidar_fov=np.radians(270),
            # Distance to which we fully trust the lidar.
            lidar_trusted_zone=0,
            **kwargs):

        self.fx = fx
        self.camera_xyz = np.asarray(camera_xyz)
        self.camera_rot = _rotation_matrix(*camera_ypr)
        self.camw, self.camh = image_size
        self.cam_low = self.camh // 2 + 1
        self.rx, self.ry = principal_point
        self.min_x = min_x
        self.min_y = min_y
        self.max_x = max_x
        self.max_y = max_y
        self.min_z = min_z
        self.max_z = max_z
        self.vertical_pixel_offset = vertical_pixel_offset
        self.vertical_diff_limit = vertical_diff_limit
        self.max_slope = max_slope
        self.slope_length_scale_factor = slope_length_scale_factor
        self.slope_length_power_factor = slope_length_power_factor
        self.noise_filter_window = noise_filter_window
        self.noise_filter_threshold = noise_filter_threshold
        self.lidar_fov = lidar_fov
        self.lidar_trusted_zone = lidar_trusted_zone

        # Pixel coordinates relative to the center of the image, with positive
        # directions to the left and up.
        pxs = self.rx - np.repeat(
                np.arange(self.camw).reshape((1, self.camw)), self.camh, axis=0)
        pys = self.ry - np.repeat(
                np.arange(self.camh).reshape((self.camh, 1)), self.camw, axis=1)
        pzs = np.ones((self.camh, self.camw), dtype=np.float)
        # For each pixel in the image, a vector representing its corresponding
        # direction in the scene with a unit forward axis.
        self.ps = (self.camera_rot @ np.dstack([pzs, pxs / fx, pys / fx]).T.reshape((3, -1))).reshape((3, self.camw, self.camh)).T


# Indices of directions in a matrix with 3D points.
X, Y, Z = 0, 1, 2


def _rotation_matrix(yaw, pitch, roll):
    # private so it will not be used by other modules -> TODO move to math or quaternion?
    cos_roll = np.cos(roll)
    sin_roll = np.sin(roll)
    roll_m = np.array([[1., 0., 0.], [0., cos_roll, -sin_roll],
                       [0., sin_roll, cos_roll]])
    cos_pitch = np.cos(pitch)
    sin_pitch = np.sin(pitch)
    pitch_m = np.array([[cos_pitch, 0, sin_pitch], [0, 1, 0],
                        [-sin_pitch, 0, cos_pitch]])
    cos_yaw = np.cos(yaw)
    sin_yaw = np.sin(yaw)
    yaw_m = np.array([[cos_yaw, -sin_yaw, 0], [sin_yaw, cos_yaw, 0],
                      [0, 0, 1]])

    return roll_m @ pitch_m @ yaw_m


def depth2danger(depth_mm, params):
    # COPY & PASTE - refactoring needed!

    depth = depth_mm * 0.001  # Converting to meters.
    # 3D coordinates of points detected by the depth camera, converted to
    # robot's coordinate system.
    xyz = (params.ps * np.expand_dims(depth, axis=Z) +
           [[params.camera_xyz]])
    # Relative positions of 3D points placed params.vertical_pixel_offset pixels
    # above each other.
    rel_xyz = (xyz[:-params.vertical_pixel_offset,:,:] -
               xyz[params.vertical_pixel_offset:,:,:])
    # Slope of a line connecting two points in the vertical plane they
    # define.
    slope = np.arctan2(
            rel_xyz[:,:,Z], np.hypot(rel_xyz[:,:,X], rel_xyz[:,:,Y]))

    nearer_x = np.minimum(xyz[:-params.vertical_pixel_offset,:,X],
                          xyz[params.vertical_pixel_offset:,:,X])
    abs_y = np.abs(xyz[:,:,Y])
    nearer_abs_y = np.minimum(abs_y[:-params.vertical_pixel_offset,:],
                              abs_y[params.vertical_pixel_offset:,:])
    danger = np.logical_and.reduce(np.stack([
        # It has to be near.
        nearer_x <= params.max_x,
        nearer_abs_y <= params.max_y,
        # But not too near.
        nearer_x >= params.min_x,
        nearer_abs_y >= params.min_y,
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


def depth2dist(depth_mm, params, pitch=None, roll=None, debug_col=None):
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
    rot = pitch_rot * roll_rot

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

    abs_y = np.abs(rxyz[:,:,Y])
    sensing_area = np.logical_and.reduce(np.stack([
        # It has to be near.
        rxyz[:-params.vertical_pixel_offset,:,X] <= params.max_x,
        rxyz[params.vertical_pixel_offset:,:,X] <= params.max_x,
        abs_y[:-params.vertical_pixel_offset,:] <= params.max_y,
        abs_y[params.vertical_pixel_offset:,:] <= params.max_y,
        # But not too near.
        rxyz[:-params.vertical_pixel_offset,:,X] >= params.min_x,
        rxyz[params.vertical_pixel_offset:,:,X] >= params.min_x,
        abs_y[:-params.vertical_pixel_offset,:] >= params.min_y,
        abs_y[params.vertical_pixel_offset:,:] >= params.min_y]))

    # Surfaces that are vertical enough to be considered not traversable.
    vertical_obstacle = np.logical_and.reduce(np.stack([
        sensing_area,
        # It should not be something small on the ground.
        np.maximum(rxyz[:-params.vertical_pixel_offset,:,Z],
                   rxyz[params.vertical_pixel_offset:,:,Z]) >= params.min_z,
        # It has to be close to vertical.
        np.abs(slope - np.radians(90)) <= params.vertical_diff_limit,
        # It should not be too high above ground.
        np.maximum(rxyz[:-params.vertical_pixel_offset, :, Z],
                   rxyz[params.vertical_pixel_offset:, :, Z]) <= params.max_z]))

    # Slopes
    rel_dists = np.hypot(rel_xyz[:,:,X], rel_xyz[:,:,Y])
    slopes = np.arctan2(rel_xyz[:,:,Z], rel_dists)
    # This threshold parametrization, with positive scale and negative power
    # factor, allows us to accept short/low steep slopes as traversable, yet
    # mark long steps with smaller slope as dangerous.
    # Short & low "steps" (=steep slopes)  arise on normal surface bumpiness,
    # while long steps show up when the robot approaches a cliff with ground at
    # a lower level below it.
    slope_thresholds = params.max_slope * ((params.slope_length_scale_factor * rel_dists) ** params.slope_length_power_factor)
    steep = np.logical_and(sensing_area, slopes < slope_thresholds)
    if debug_col is not None:
        for info in enumerate(
                zip(np.degrees(slopes[:,debug_col]),
                    rel_dists[:,debug_col],
                    np.degrees(slope_thresholds[:,debug_col]),
                    rel_xyz[:,debug_col,:].tolist(),
                    np.hypot(gxyz[params.vertical_pixel_offset:,debug_col,X],
                    gxyz[params.vertical_pixel_offset:,debug_col,Y]),
                    depth[params.vertical_pixel_offset:,debug_col],
                    steep[:,debug_col])):
            print(info)
        print(np.any(steep[:,debug_col]), steep.shape[0] - np.argmax(steep[::-1,debug_col]) - 1)

    noisy_danger = np.logical_or(vertical_obstacle, steep)

    # Filter out noise.
    danger = cv2.filter2D(
            noisy_danger.astype(np.float),
            -1,
            np.ones(params.noise_filter_window)) > params.noise_filter_threshold

    dists = np.hypot(rxyz[:,:,X], rxyz[:,:,Y])
    FAR_AWAY = 1000.0
    dists = np.where(danger, dists[params.vertical_pixel_offset:,:], FAR_AWAY)
    scan = np.min(dists, axis=0)
    mask = scan == FAR_AWAY
    scan[mask] = 0

    scan = np.array(scan*1000, dtype=np.int32)
    return scan


if __name__ == '__main__':
    import argparse
    import cv2
    import json
    import matplotlib.pyplot as plt
    from osgar.lib.quaternion import euler_zyx

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('filename', help='NPZ file with depth data')
    parser.add_argument('--depth-params', type=json.loads, default={},
                        help='Depth processing parameters in json format. ' +
                              'See DepthParams.')
    parser.add_argument('--debug-column', type=int,
                        help='Print extra debugging information for this column in the image.')
    args = parser.parse_args()

    with np.load(args.filename, allow_pickle=True) as f:
        depth = f['depth']
        try:
            xyz, rot = f['pose3d']
            yaw, pitch, roll = euler_zyx(rot)

            # Mimicking Osgar's upside down bug.
            pitch = -pitch
            roll = -roll - np.pi
        except:
            assert(False)
            pitch = 0.0
            roll = 0.0

        try:
            img = f['img']
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        except:
            img = None

    depth_params = DepthParams(**args.depth_params)
    dist = depth2dist(depth, depth_params, pitch, roll, debug_col=args.debug_column)

    plt.figure(1)
    plt.plot(range(640), dist, 'o-', linewidth=2)

    depth_img = (np.minimum(255*40, depth) / 40).astype(np.uint8)
    depth_im_color = cv2.applyColorMap(depth_img, cv2.COLORMAP_JET)
    plt.figure(2)
    plt.imshow(depth_im_color)

    if img is not None:
        plt.figure(3)
        plt.imshow(img)

    plt.show()

# vim: expandtab sw=4 ts=4
