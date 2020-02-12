"""
  pyrealsense2 OSGAR wrapper

"""
import math
import threading
import numpy as np

try:
    import pyrealsense2 as rs
    if rs.pyrealsense2.__version__.startswith('2.32'):
        print(f"RealSense version {rs.pyrealsense2.__version__} is broken in several ways!")
except:
    print('RealSense not installed!')
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
        bus.register('pose2d', 'pose3d', 'pose_raw', 'orientation', 'depth')
        self.verbose = config.get('verbose', False)
        self.pose_pipeline = None  # not initialized yet
        self.depth_pipeline = None
        self.finished = None

    def pose_callback(self, frame):
        # TODO: add decimation based on config from 200Hz to desired value
        try:
            pose = frame.as_pose_frame().get_pose_data()
            n = frame.get_frame_number()
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
        # TODO: add decimation based on config from 200Hz to desired value
        try:
            assert frameset.is_frameset()
            frame = frameset.as_frameset().get_depth_frame()
            assert frame.is_depth_frame()
            depth_image = np.asanyarray(frame.as_depth_frame().get_data())
        except Exception as e:
            print(e)
            self.finished.set()
            return
        self.publish('depth', depth_image)

    def start(self):
        self.finished = threading.Event()
        ctx = rs.context()
        device_list = ctx.query_devices()
        if len(device_list) == 0:
            print("No RealSense devices detected!")
            self.finished.set()
            return

        enable_pose, enable_depth = False, False
        for device in device_list:
            name = device.get_info(rs.camera_info.name)
            serial_number = device.get_info(rs.camera_info.serial_number)
            print(f"Found {name} (S/N: {serial_number}):", end=' ')
            product_line = device.get_info(rs.camera_info.product_line)
            if product_line == "D400":
                print("Enabling depth stream")
                enable_depth = True
            elif product_line == "T200":
                enable_pose = True
                print("Enabling pose stream")
            else:
                print("Unknown type")

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
