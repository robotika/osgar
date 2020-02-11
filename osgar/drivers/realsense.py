"""
  pyrealsense2 OSGAR wrapper

"""
import math
import numpy as np

try:
    import pyrealsense2 as rs
    import pkg_resources
    rs_version = pkg_resources.get_distribution("pyrealsense2").version
    if rs_version.startswith('2.32'):
        print(f"RealSense version {rs_version} is broken in several ways!")
except:
    print('RealSense not installed!')
    from unittest.mock import MagicMock
    rs = MagicMock()

from osgar.node import Node
from osgar.bus import BusShutdownException
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

    def poll_pose(self, tick):
        frames = self.pose_pipeline.wait_for_frames()
        if len(frames) == 0:
            print(f"{tick}: No pose frames available")
            return

        pose_frame = frames.get_pose_frame()
        assert frames[0].is_pose_frame()
        pose = frames[0].as_pose_frame().get_pose_data()
        n = pose_frame.get_frame_number()
        timestamp = pose_frame.get_timestamp()
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

    def poll_depth(self, tick):
        frames = self.depth_pipeline.poll_for_frames()
        if len(frames) == 0:
            print(f"{tick}: No depth frames available")
            return

        depth_frame = frames.get_depth_frame()
        assert depth_frame.is_depth_frame()
        depth_image = np.asanyarray(depth_frame.get_data())
        self.publish('depth', depth_image)

    def run(self):
        ctx = rs.context()
        device_list = ctx.query_devices()
        if len(device_list) == 0:
            print("No RealSense devices detected!")
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
            self.pose_pipeline.start(pose_cfg)

        if enable_depth:
            self.depth_pipeline = rs.pipeline(ctx)
            depth_cfg = rs.config()
            depth_cfg.enable_stream(rs.stream.depth, 640, 360, rs.format.z16, 30)
            self.depth_pipeline.start(depth_cfg)

        try:
            tick = 0
            while True:
                channel = self.update()
                if channel == 'trigger':
                    if enable_pose: self.poll_pose(tick)
                    if enable_depth: self.poll_depth(tick)
                    tick += 1
        except BusShutdownException:
            pass
        self.pose_pipeline.stop()


# vim: expandtab sw=4 ts=4
