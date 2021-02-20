"""
  pyrealsense2 OSGAR wrapper

"""
import math
import logging
import threading
import numpy as np

g_logger = logging.getLogger(__name__)

try:
    import pyrealsense2 as rs
    try:
        prefix = rs.pyrealsense2.__version__[:4]
        if prefix in ('2.32', '2.33'):
            g_logger.error(f"RealSense version {rs.pyrealsense2.__version__} is broken in several ways!")
    except Exception as e:
        g_logger.error('pyrealsense2 version not found due to: ' + str(e))
except:
    g_logger.info('RealSense not installed!')
    from unittest.mock import MagicMock
    rs = MagicMock()

from osgar.node import Node
from osgar.lib import quaternion


# https://github.com/IntelRealSense/librealsense/blob/master/doc/t265.md#sensor-origin-and-coordinate-system

def t265_to_osgar_position(t265_position):
    x = -t265_position.z
    y = -t265_position.x
    z = t265_position.y
    return [x, y, z]


def t265_to_osgar_orientation(t265_orientation):
    x0 = t265_orientation.z
    y0 = -t265_orientation.x
    z0 = t265_orientation.y
    w0 = t265_orientation.w
    return [x0, y0, z0, w0]


class RealSense(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        self.device = config.get('device')

        if self.device == 'T200':
            bus.register('pose2d', 'pose3d', 'pose_raw', 'orientation')
            self.pose_subsample = config.get("pose_subsample", 20)
        elif self.device in ['D400', 'L500']:
            bus.register('depth:gz', 'color', 'infra')
            self.depth_subsample = config.get("depth_subsample", 3)
            self.depth_rgb = config.get("depth_rgb", False)
            self.depth_infra = config.get("depth_infra", False)
            self.default_depth_resolution = config.get("depth_resolution", [640, 360])
            self.default_rgb_resolution = config.get("rgb_resolution", [640, 360])
            self.depth_fps = config.get("depth_fps", 30)

            if self.depth_rgb or self.depth_infra:
                import cv2
                global cv2
        else:
            g_logger.warning("Device is not specified in the config!")

        self.ser_number = config.get('ser_number')
        self.verbose = config.get('verbose', False)
        self.pose_pipeline = None  # not initialized yet
        self.depth_pipeline = None
        self.finished = None

    def pose_callback(self, frame):
        try:
            pose = frame.as_pose_frame().get_pose_data()
            n = frame.get_frame_number()
            if n % self.pose_subsample != 0:
                return
            timestamp = frame.get_timestamp()
        except Exception as e:
            print(e)
            self.finished.set()
            return
        orientation = t265_to_osgar_orientation(pose.rotation)
        self.publish('orientation', orientation)
        x, y, z = t265_to_osgar_position(pose.translation)
        yaw = quaternion.heading(orientation)
        self.publish('pose2d', [int(x * 1000), int(y * 1000), int(math.degrees(yaw) * 100)])
        self.publish('pose3d', [[x, y, z], orientation])
        self.publish('pose_raw', [n, timestamp,
                             [pose.translation.x, pose.translation.y, pose.translation.z],
                             [pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w],
                             [pose.velocity.x, pose.velocity.y, pose.velocity.z],
                             [pose.angular_velocity.x, pose.angular_velocity.y, pose.angular_velocity.z],
                             [pose.acceleration.x, pose.acceleration.y, pose.acceleration.z],
                             [pose.angular_acceleration.x, pose.angular_acceleration.y,
                              pose.angular_acceleration.z],
                             [pose.mapper_confidence, pose.tracker_confidence],
                             ])  # raw RealSense2 Pose data

    def depth_callback(self, frameset):
        try:
            assert frameset.is_frameset()
            depth_frame = frameset.as_frameset().get_depth_frame()
            n = depth_frame.get_frame_number()
            if n % self.depth_subsample != 0:
                return
            assert depth_frame.is_depth_frame()
            depth_image = np.asanyarray(depth_frame.as_depth_frame().get_data())

            if self.depth_rgb:
                color_frame = frameset.as_frameset().get_color_frame()
                assert color_frame.is_video_frame()
                color_image = np.asanyarray(color_frame.as_video_frame().get_data())
                __, data = cv2.imencode('*.jpeg', color_image)

            if self.depth_infra:
                infra_frame = frameset.as_frameset().get_infrared_frame()
                assert infra_frame.is_video_frame()
                infra_image = np.asanyarray(infra_frame.as_video_frame().get_data())
                __, infra_data = cv2.imencode('*.jpeg', infra_image)

        except Exception as e:
            print(e)
            self.finished.set()
            return

        self.publish('depth', depth_image)
        if self.depth_rgb:
            self.publish('color', data.tobytes())
        if self.depth_infra:
            self.publish('infra', infra_data.tobytes())


    def start(self):
        self.finished = threading.Event()
        ctx = rs.context()
        if self.device in ["D400", "L500"]:
            info_msg = "Enabling streams: depth"
            if self.depth_rgb:
                info_msg += ", depth_rgb"
            if self.depth_infra:
                info_msg += ", depth_infra"
            g_logger.info(info_msg)

            self.depth_pipeline = rs.pipeline(ctx)
            depth_cfg = rs.config()
            if self.ser_number:
                depth_cfg.enable_device(self.ser_number)
            w, h = self.default_depth_resolution
            depth_cfg.enable_stream(rs.stream.depth, w, h, rs.format.z16, self.depth_fps)
            if self.depth_rgb:
                w, h = self.default_rgb_resolution
                depth_cfg.enable_stream(rs.stream.color, w, h, rs.format.bgr8, self.depth_fps)
            if self.depth_infra:
                w, h = self.default_depth_resolution
                depth_cfg.enable_stream(rs.stream.infrared, w, h, rs.format.y8, self.depth_fps)
            self.depth_pipeline.start(depth_cfg, self.depth_callback)

        elif self.device == "T200":
            g_logger.info("Enabling pose stream")
            self.pose_pipeline = rs.pipeline(ctx)
            pose_cfg = rs.config()
            if self.ser_number:
                pose_cfg.enable_device(self.ser_number)
            pose_cfg.enable_stream(rs.stream.pose)
            self.pose_pipeline.start(pose_cfg, self.pose_callback)

        else:
            self.finished.set()

    def request_stop(self):
        if self.pose_pipeline is not None:
            self.pose_pipeline.stop()
        if self.depth_pipeline is not None:
            self.depth_pipeline.stop()
        super().request_stop()
        self.finished.set()

    def join(self, timeout_sec=None):
        self.finished.wait(timeout_sec)


# vim: expandtab sw=4 ts=4
