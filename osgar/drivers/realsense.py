"""
  pyrealsense2 OSGAR wrapper

"""
from collections import namedtuple
from functools import partial

try:
    import pyrealsense2 as rs
except:
    print('RealSense not installed!')
    from unittest.mock import MagicMock
    rs = MagicMock()

from osgar.node import Node
from osgar.bus import BusShutdownException


attrs = [
    'acceleration',
    'angular_acceleration',
    'angular_velocity',
    'mapper_confidence',
    'rotation',
    'tracker_confidence',
    'translation',
    'velocity',
    ]
Pose = namedtuple('Pose', attrs)


class RealSense(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('pose2d', 'raw')
        self.verbose = config.get('verbose', False)
        self.pipeline = None  # not initialized yet

    def update(self):
        channel = super().update()  # define self.time
        if channel == 'trigger':
            frames = self.pipeline.wait_for_frames()
            pose_frame = frames.get_pose_frame()
            if pose_frame:
                pose = pose_frame.get_pose_data()
                n = pose_frame.get_frame_number()
                timestamp = pose_frame.get_timestamp()
                p = Pose(*map(partial(getattr, pose), attrs))
                x = pose.translation.z
                y = pose.translation.x  # not sure yet
                self.publish('pose2d', [int(x*1000), int(y*1000), 0])  # TODO heading
                self.publish('raw', [n, timestamp, 
                    [pose.translation.x, pose.translation.y, pose.translation.z],
                    [pose.rotation.w, pose.rotation.x, pose.rotation.y, pose.rotation.z],
                    [pose.velocity.x, pose.velocity.y, pose.velocity.z],
                    [pose.angular_velocity.x, pose.angular_velocity.y, pose.angular_velocity.z],
                    [pose.acceleration.x, pose.acceleration.y, pose.acceleration.z],
                    [pose.angular_acceleration.x, pose.angular_acceleration.y, pose.angular_acceleration.z],
                    [pose.mapper_confidence, pose.tracker_confidence],
                    ])  # raw RealSense2 Pose data
        return channel

    def run(self):
        self.pipeline = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.pose)
        self.pipeline.start(cfg)
        try:
            while True:
                self.update()
        except BusShutdownException:
            pass
        self.pipeline.stop()


# vim: expandtab sw=4 ts=4
