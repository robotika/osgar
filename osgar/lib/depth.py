"""
  Convert depth images to B&W where White is danger
"""

import cv2
import numpy as np

from osgar.lib.serialize import deserialize


class DepthParams:
    def __init__(
            self,
            # Focal length.
            fx=462.1,
            # Placement of the camera relative to the center of the robot.
            camera_xyz=(0.23, 0, (0.19 + 0.06256005)),
            # Image dimensions.
            image_size=(640, 360),
            # Principal point, position of optical axis
            principal_point=(640 / 2. +  0.5, 360 / 2 + 0.5),
            # An integer downscaling factor. All oter parameters, such as
            # image dimensions, focal length etc. match the image before
            # downscaling.
            # This value can be either a single integer, or a pair of integers
            # for a different horizontal and vertical stride.
            stride = 1,
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
        try:
            self.horizontal_stride, self.vertical_stride = stride
        except:
            self.horizontal_stride = self.vertical_stride = stride
        self.fx = fx / self.horizontal_stride
        self.fy = fx / self.vertical_stride
        self.camera_xyz = np.asarray(camera_xyz)
        self.camw, self.camh = image_size
        self.camw //= self.horizontal_stride
        self.camh //= self.vertical_stride
        self.cam_low = self.camh // 2 + 1
        self.rx, self.ry = principal_point
        self.rx /= self.horizontal_stride
        self.ry /= self.vertical_stride
        self.min_x = min_x
        self.min_y = min_y
        self.max_x = max_x
        self.max_y = max_y
        self.min_z = min_z
        self.max_z = max_z
        self.vertical_pixel_offset = max(
                1, vertical_pixel_offset // self.vertical_stride)
        self.vertical_diff_limit = vertical_diff_limit
        self.max_slope = max_slope
        self.slope_length_scale_factor = slope_length_scale_factor
        self.slope_length_power_factor = slope_length_power_factor
        self.noise_filter_window = [
                max(1, v // s) for (v, s) in
                zip(noise_filter_window,
                    [self.horizontal_stride, self.vertical_stride])]
        self.noise_filter_threshold = max(
                1,
                noise_filter_threshold //
                    (self.horizontal_stride * self.vertical_stride))
        self.lidar_fov = lidar_fov
        self.lidar_trusted_zone = lidar_trusted_zone

        # Pixel coordinates relative to the center of the image, with positive
        # directions to the left and up.
        pxs = self.rx - np.repeat(
                np.arange(self.camw).reshape((1, self.camw)), self.camh, axis=0)
        pys = self.ry - np.repeat(
                np.arange(self.camh).reshape((self.camh, 1)), self.camw, axis=1)
        pzs = np.ones((self.camh, self.camw), dtype=float)
        # For each pixel in the image, a vector representing its corresponding
        # direction in the scene with a unit forward axis.
        self.ps = np.dstack([pzs, pxs / self.fx, pys / self.fy]).T.reshape((3, -1)).reshape((3, self.camw, self.camh)).T


# Indices of directions in a matrix with 3D points.
X, Y, Z = 0, 1, 2


def depth2danger(depth, params):
    # COPY & PASTE - refactoring needed!

    # Make the input depth image smaller, if requested.
    orig_shape = depth.shape
    if params.horizontal_stride != 1 or params.vertical_stride != 1:
        depth = depth[::params.vertical_stride, ::params.horizontal_stride]

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
            danger.astype(float),
            -1,
            np.ones(params.noise_filter_window)) > params.noise_filter_threshold

    if danger.shape != orig_shape:
        danger = cv2.resize(
                danger.astype(np.uint8),
                (orig_shape[1], orig_shape[0]),
                interpolation=cv2.INTER_NEAREST).astype(bool)

    return danger


def depth2dist(depth, params, pitch=None, roll=None, yaw=None, debug_col=None):
    # return line in mm corresponding to scan
    # optional pitch and roll angles are in radians
    yaw_rot = np.asmatrix(np.eye(3))
    pitch_rot = np.asmatrix(np.eye(3))
    roll_rot = np.asmatrix(np.eye(3))
    # http://planning.cs.uiuc.edu/node102.html
    if yaw is not None:
        yaw_rot = np.matrix(
                [[np.cos(yaw), -np.sin(yaw), 0.0],
                 [np.sin(yaw), np.cos(yaw), 0.0],
                 [0.0, 0.0, 1.0]])
    if pitch is not None:
        pitch_rot = np.matrix(
                [[np.cos(pitch), 0.0, np.sin(pitch)],
                 [0.0, 1.0, 0.0],
                 [-np.sin(pitch), 0.0, np.cos(pitch)]])
    if roll is not None:
        roll_rot = np.matrix(
                [[1.0, 0.0, 0.0],
                 [0.0, np.cos(roll), -np.sin(roll)],
                 [0.0, np.sin(roll), np.cos(roll)]])
    rot = yaw_rot * pitch_rot * roll_rot

    # Make the input depth image smaller, if requested.
    orig_width = depth.shape[1]
    if params.horizontal_stride != 1 or params.vertical_stride != 1:
        depth = depth[::params.vertical_stride, ::params.horizontal_stride]

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
            noisy_danger.astype(float),
            -1,
            np.ones(params.noise_filter_window)) > params.noise_filter_threshold

    dists = np.hypot(rxyz[:,:,X], rxyz[:,:,Y])
    FAR_AWAY = 1000.0
    dists = np.where(danger, dists[params.vertical_pixel_offset:,:], FAR_AWAY)
    scan = np.min(dists, axis=0)
    mask = scan == FAR_AWAY
    scan[mask] = 0

    if scan.shape[0] != orig_width:
        scan = cv2.resize(scan[np.newaxis,:],
                          (orig_width, 1),
                          interpolation=cv2.INTER_NEAREST)[0,:]

    scan = np.array(scan*1000, dtype=np.int32)

    return scan


def decompress(data):
    # RTABMap compresses float32 depth data into RGBA channes of a PNG image.
    # It does not, however, store information about endiannes. All we can do is
    # hope that the current machine and the data origin machine have the same
    # one.
    depth_img = cv2.imdecode(np.frombuffer(data, np.uint8),
                        cv2.IMREAD_UNCHANGED)
    if depth_img is not None:
        return depth_img.view(dtype=np.float32)[:,:,0]
    return deserialize(data)


def compress(depth):
    return cv2.imencode('.png', depth.view(np.uint8))[1].tobytes()


def depth_to_rgb_align(depth_ar, depth_intr, rgb_intr, T, rgb_shape):
    """
        Aligns the depth to the RGB image. Calculation:
            Deprojection the depth pixel to 3D poit in depth frame
            x = (u - cx_d) * z / fx_d
            y = (v - cy_d) * z / fy_d
            point_d = [x, y, z, 1]

            Convert to RGB frame
            point_rgb = T @ point_d

            x_rgb, y_rgb, z_rgb, __ = point_rgb

            Projection to the RGB frame
            u_rgb = (x_rgb * fx_rgb) / z_rgb + cx_rgb
            v_rgb = (y_rgb * fy_rgb) / z_rgb + cy_rgb

        Args:
            depth_ar (np.array): depth in mm
            depth_intr and rgb_intr (list of lists): matrix = [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]
            T (np.array): Transformation matrix T = [[r00, r01, r02, tx],
                                                     [r10, r11, r12, ty],
                                                     [r20, r21, r22, tz],
                                                     [0,   0,   0,   1]]
            rgb_shape (list): desired rgb shape

        Note:
            For oak-d cameras:
                https://docs.luxonis.com/hardware/platform/depth/calibration
                https://docs.luxonis.com/software/depthai/examples/calibration_reader/
                Use transformation matrix for right mono camera.
    """
    height_d, width_d = depth_ar.shape
    height_rgb, width_rgb, __ = rgb_shape
    fx_d, fy_d = depth_intr[0][0], depth_intr[1][1]
    cx_d, cy_d = depth_intr[0][2], depth_intr[1][2]

    fx_rgb, fy_rgb = rgb_intr[0][0], rgb_intr[1][1]
    cx_rgb, cy_rgb = rgb_intr[0][2], rgb_intr[1][2]

    align_depth = np.zeros((height_rgb, width_rgb), dtype=np.uint16)

    # Creating coordinate grids for depth image
    u_d, v_d = np.meshgrid(np.arange(width_d), np.arange(height_d))

    # Transfer depth data to meters and set it as z coordinates
    z = depth_ar[v_d, u_d] / 1000.0

    # Mask for non zero depth
    valid_depth_mask = z > 0

    # Calculation of 3D points in the depth camera coordinate system for valid depths
    x_d = (u_d[valid_depth_mask] - cx_d) * z[valid_depth_mask] / fx_d
    y_d = (v_d[valid_depth_mask] - cy_d) * z[valid_depth_mask] / fy_d
    z_valid = z[valid_depth_mask]
    ones = np.ones_like(z_valid)
    points_d = np.stack([x_d, y_d, z_valid, ones], axis=-1)

    # Transforming 3D points into the RGB camera coordinate system
    points_rgb = points_d @ T.T  # We use T.T for correct multiplication of matrices with vectors as rows

    x_rgb = points_rgb[:, 0]
    y_rgb = points_rgb[:, 1]
    z_rgb = points_rgb[:, 2]

    # Masking points behind an RGB camera
    valid_rgb_z_mask = z_rgb > 0

    x_rgb_valid = x_rgb[valid_rgb_z_mask]
    y_rgb_valid = y_rgb[valid_rgb_z_mask]
    z_rgb_valid = z_rgb[valid_rgb_z_mask]

    # Projection of 3D points into the image plane of an RGB camera
    u_rgb = np.round((x_rgb_valid * fx_rgb) / z_rgb_valid + cx_rgb).astype(int)
    v_rgb = np.round((y_rgb_valid * fy_rgb) / z_rgb_valid + cy_rgb).astype(int)
    z_mm = np.round(z_rgb_valid * 1000).astype(int)

    # Masking points outside the RGB range of an image
    valid_rgb_coord_mask = (u_rgb >= 0) & (u_rgb < width_rgb) & (v_rgb >= 0) & (v_rgb < height_rgb)

    u_rgb_valid_in_frame = u_rgb[valid_rgb_coord_mask]
    v_rgb_valid_in_frame = v_rgb[valid_rgb_coord_mask]
    z_mm_valid_in_frame = z_mm[valid_rgb_coord_mask]

    # Updating align_depth using NumPy indexing
    align_depth_flat_indices = np.ravel_multi_index((v_rgb_valid_in_frame, u_rgb_valid_in_frame), align_depth.shape)

    # Creating an array for new depths and initializing to zero
    new_depths = np.zeros(align_depth.size, dtype=align_depth.dtype)
    np.put(new_depths, align_depth_flat_indices, z_mm_valid_in_frame)
    new_depths_reshaped = new_depths.reshape(align_depth.shape)

    # Update align_depth only where the new depth is smaller or the original is 0
    update_mask = (new_depths_reshaped < align_depth) | (align_depth == 0)
    align_depth[update_mask] = new_depths_reshaped[update_mask]

    return align_depth


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
