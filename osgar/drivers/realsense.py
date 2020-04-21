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
        bus.register('pose2d', 'pose3d', 'pose_raw', 'orientation', 'depth', 'depth_rgb')
        self.verbose = config.get('verbose', False)
        self.depth_subsample = config.get("depth_subsample", 3)
        self.pose_subsample = config.get("pose_subsample", 20)
        self.depth_rgb_subsample = config.get("image_subsample", 3)
        self.pose_pipeline = None  # not initialized yet
        self.depth_pipeline = None
        self.depth_rgb_pipeline = None
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
            frame = frameset.as_frameset().get_depth_frame()
            n = frame.get_frame_number()
            if n % self.depth_subsample != 0:
                return
            assert frame.is_depth_frame()
            depth_image = np.asanyarray(frame.as_depth_frame().get_data())
        except Exception as e:
            print(e)
            self.finished.set()
            return
        self.publish('depth', depth_image)

    def depth_rgb_callback(self, frameset):
        try:
            assert frameset.is_frameset()
            frame = frameset.as_frameset().get_color_frame()
            n = frame.get_frame_number()
            if n % self.depth_rgb_subsample != 0:
                return
            #assert frame.is_color_frame()
            depth_rgb = np.asanyarray(frame.as_depth_frame().get_data())
        except Exception as e:
            print(e)
            self.finished.set()
            return
        self.publish('depth_rgb', depth_rgb)


    def start(self):
        self.finished = threading.Event()
        ctx = rs.context()
        device_list = ctx.query_devices()
        if len(device_list) == 0:
            g_logger.warning("No RealSense devices detected!")
            self.finished.set()
            return

        enable_pose, enable_depth, enable_depth_rgb = False, False, False
        for device in device_list:
            name = device.get_info(rs.camera_info.name)
            serial_number = device.get_info(rs.camera_info.serial_number)
            intro = f"Found {name} (S/N: {serial_number}): "
            product_line = device.get_info(rs.camera_info.product_line)
            if product_line == "D400":
                g_logger.info(intro + "Enabling depth stream")
                enable_depth = True
                enable_depth_rgb = True
            elif product_line == "T200":
                enable_pose = True
                g_logger.info(intro + "Enabling pose stream")
            else:
                g_logger.warning(f"Unknown type: {product_line}")

        if enable_pose:
            self.pose_pipeline = rs.pipeline(ctx)
            pose_cfg = rs.config()
            pose_cfg.enable_stream(rs.stream.pose)
            self.pose_pipeline.start(pose_cfg, self.pose_callback)

        if enable_depth:
            self.depth_pipeline = rs.pipeline(ctx)
            depth_cfg = rs.config()
            depth_cfg.enable_stream(rs.stream.depth, 640, 360, rs.format.z16, 30)
            self.depth_pipeline.start(depth_cfg, self.depth_callback)

        if enable_depth_rgb:
            self.depth_rgb_pipeline = rs.pipeline(ctx)
            depth_rgb_cfg = rs.config()
            depth_rgb_cfg.enable_stream(rs.stream.color, 640, 360, rs.format.bgr8, 30)
            self.depth_rgb_pipeline.start(depth_rgb_cfg, self.depth_rgb_callback)

        if not enable_pose and not enable_depth:
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
