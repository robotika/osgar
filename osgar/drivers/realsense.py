"""
  pyrealsense2 OSGAR wrapper

"""
import math

try:
    import pyrealsense2 as rs
    import pkg_resources
    rs_version = pkg_resources.get_distribution("pyrealsense2").version
    if rs_version.startswith('2.32'):
        print(f"RealSense version {rs_version} does not support T265 multicam!")
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
        bus.register('pose2d', 'pose3d', 'raw', 'orientation', 'depth')
        self.verbose = config.get('verbose', False)
        self.pose_pipeline = None  # not initialized yet

    def update(self):
        channel = super().update()  # define self.time
        if channel == 'trigger':
            frames = self.pose_pipeline.wait_for_frames()
            pose_frame = frames.get_pose_frame()
            if pose_frame:
                pose = pose_frame.get_pose_data()
                n = pose_frame.get_frame_number()
                timestamp = pose_frame.get_timestamp()
                orientation = t265_to_osgar_orientation(pose.rotation)
                self.publish('orientation', orientation)
                x, y, z = t265_to_osgar_position(pose.translation)
                yaw = quaternion.heading(orientation)
                self.publish('pose2d', [int(x*1000), int(y*1000), int(math.degrees(yaw)*100)])
                self.publish('pose3d', [[x, y, z], orientation])
                self.publish('raw', [n, timestamp,
                    [pose.translation.x, pose.translation.y, pose.translation.z],
                    [pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w],
                    [pose.velocity.x, pose.velocity.y, pose.velocity.z],
                    [pose.angular_velocity.x, pose.angular_velocity.y, pose.angular_velocity.z],
                    [pose.acceleration.x, pose.acceleration.y, pose.acceleration.z],
                    [pose.angular_acceleration.x, pose.angular_acceleration.y, pose.angular_acceleration.z],
                    [pose.mapper_confidence, pose.tracker_confidence],
                    ])  # raw RealSense2 Pose data
        return channel

    def run(self):
        self.pose_pipeline = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.pose)
        self.pose_pipeline.start(cfg)
        try:
            while True:
                self.update()
        except BusShutdownException:
            pass
        self.pose_pipeline.stop()


# vim: expandtab sw=4 ts=4
