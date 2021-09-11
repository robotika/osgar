import math

import cv2
import numpy as np

from osgar.lib.quaternion import euler_to_quaternion
from osgar.node import Node


class Bundler(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        self.rgb_params = config['rgb']
        self.depth_params = config['depth']
        self.extrinsic_params = np.asarray(
                config.get('extrinsics', [[1.0, 0.0, 0.0, 0.0],
                                          [0.0, 1.0, 0.0, 0.0],
                                          [0.0, 0.0, 1.0, 0.0]]))
        camera_config = config.get('camera')
        if camera_config is not None:
            self.camera_pose = (
                    camera_config['xyz'],
                    euler_to_quaternion(
                        *[math.radians(x) for x in camera_config['ypr']]))
        else:
            self.camera_pose = None
        self.robot_pose = None
        self.img = None
        self.depth = None

        bus.register('rgbd:gzip')

        # Pixel coordinates relative to the center of the image, with positive
        # directions to the right and up.
        pxs = np.repeat(
                np.arange(self.depth_params['w']).reshape(
                    (1, self.depth_params['w'])),
                self.depth_params['h'], axis=0) - self.depth_params['cx']
        pys = self.depth_params['cy'] - np.repeat(
                np.arange(self.depth_params['h']).reshape(
                    (self.depth_params['h'], 1)),
                self.depth_params['w'], axis=1)
        pzs = np.ones((self.depth_params['h'], self.depth_params['w']),
                      dtype=np.float)
        # For each pixel in the image, a vector representing its corresponding
        # direction in the depth scene with a unit forward axis.
        self.ps = np.dstack([pxs / self.depth_params['fx'],
                             pys / self.depth_params['fy'],
                             pzs])
        self.ones = np.ones((self.depth_params['h'], self.depth_params['w']))

    def update(self):
        timestamp, channel, data = self.bus.listen()
        setattr(self, channel, data)
        if (self.robot_pose is not None and
            self.camera_pose is not None and
            self.img is not None
            and self.depth is not None):
            self.publish(
                    'rgbd',
                    [self.robot_pose,
                     self.camera_pose,
                     self.img,
                     self.reproject(self.depth).tobytes()])
        return channel

    def reproject(self, depth):
        depth = depth / 1000.0
        depth_xyz = self.ps * np.expand_dims(depth, axis=-1)
        img_xyz = (self.extrinsic_params @ np.dstack([
                depth_xyz, self.ones]).reshape((-1, 4)).T).T
        # Projecting to a small image first and upscaling later to avoid having
        # lots of pixels without any projected depth value.
        img_u = self.rgb_params['cx'] / 2 + (self.rgb_params['fx'] / 2) * img_xyz[:,0] / img_xyz[:,2]
        img_v = self.rgb_params['cy'] / 2 - (self.rgb_params['fy'] / 2) * img_xyz[:,1] / img_xyz[:,2]
        img_u = np.round(img_u).astype(int)
        img_v = np.round(img_v).astype(int)
        visible = np.logical_and.reduce(np.stack([
            img_u >= 0,
            img_u < self.rgb_params['w'] // 2,
            img_v >= 0,
            img_v < self.rgb_params['h'] // 2]))
        reprojected = np.zeros((self.rgb_params['h'] // 2,
                                self.rgb_params['w'] // 2))
        reprojected[img_v[visible], img_u[visible]] = img_xyz[visible, 2]
        reprojected *= 1000
        # Upscale.
        reprojected = cv2.resize(reprojected, (self.rgb_params['w'], self.rgb_params['h']))
        reprojected = reprojected.astype(np.uint16)

        return reprojected

