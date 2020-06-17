import unittest
from unittest.mock import patch, MagicMock
import datetime
from collections import namedtuple

import numpy as np
import cv2

from osgar.drivers.realsense import RealSense
from osgar.bus import Bus


Pose = namedtuple('Pose', [
    'acceleration',
    'angular_acceleration',
    'angular_velocity',
    'mapper_confidence',
    'rotation',
    'tracker_confidence',
    'translation',
    'velocity',
    ])
Rotation = namedtuple('Rotation', ['x', 'y', 'z', 'w'])
Translation = namedtuple('Translation', ['x', 'y','z'])
Velocity = namedtuple('Velocity', ['x', 'y','z'])
AngularVelocity = namedtuple('AngularVelocity', ['x', 'y','z'])
Acceleration = namedtuple('Acceleration', ['x', 'y','z'])
AngularAcceleration = namedtuple('AngularAcceleration', ['x', 'y','z'])


class RealSenseTest(unittest.TestCase):
    def test_detect_pose(self):
        bus = MagicMock()
        with patch('osgar.drivers.realsense.rs') as rs:
            ctx = rs.context.return_value
            device = MagicMock()
            device.get_info.return_value = "T200"
            ctx.query_devices.return_value = [device]
            c = RealSense(bus=bus.handle('rs'), config={})
            c.start()
            self.assertTrue(c.pose_pipeline.start.called)
            self.assertEqual(c.depth_pipeline, None)
            c.request_stop()
            c.join()

    def test_detect_depth(self):
        bus = MagicMock()
        with patch('osgar.drivers.realsense.rs') as rs:
            ctx = rs.context.return_value
            device = MagicMock()
            device.get_info.return_value = "D400"
            ctx.query_devices.return_value = [device]
            c = RealSense(bus=bus.handle('rs'), config={})
            c.start()
            self.assertTrue(c.depth_pipeline.start.called)
            self.assertEqual(c.pose_pipeline, None)
            c.request_stop()
            c.join()

    def test_detect_pose_depth(self):
        bus = MagicMock()
        with patch('osgar.drivers.realsense.rs') as rs:
            ctx = rs.context.return_value
            pose_device = MagicMock()
            pose_device.get_info.return_value = "T200"
            depth_device = MagicMock()
            depth_device.get_info.return_value = "D400"
            ctx.query_devices.return_value = [pose_device, depth_device]
            c = RealSense(bus=bus.handle('rs'), config={})
            c.start()
            self.assertTrue(c.pose_pipeline.start.called)
            self.assertTrue(c.depth_pipeline.start.called)
            c.request_stop()
            c.join()

    def test_pose2d_moves(self):
        logger = MagicMock()
        logger.write = MagicMock(return_value=datetime.timedelta(microseconds=9721))
        bus = Bus(logger)
        moves = [
            [Translation(0, 0, -1), [1000, 0, 0]], # forward
            [Translation(-1, 0, 0), [0, 1000, 0]], # left
        ]
        c = RealSense(bus=bus.handle('rs'), config={})
        tester = bus.handle('tester')
        bus.connect('rs.pose2d', 'tester.pose2d')

        for input, output in moves:
            frame = MagicMock()
            frame.get_frame_number.return_value = 1
            frame.get_timestamp.return_value = 1
            pose_frame = frame.as_pose_frame.return_value
            pose_frame.get_pose_data.return_value = Pose(
                Acceleration(0, 0, 0),
                AngularAcceleration(0, 0, 0),
                AngularVelocity(0, 0, 0),
                0,
                Rotation(0, 0, 0, 1),
                0,
                input,
                Velocity(0, 0, 0))
            frame.get_timestamp.return_value = 0
            frame.get_frame_number.return_value = 0
            c.pose_callback(frame)
            dt, channel, pose2d = tester.listen()
            self.assertEqual(channel, 'pose2d')
            self.assertEqual(pose2d, output)

    def test_orientation(self):
        logger = MagicMock()
        logger.write = MagicMock(return_value=datetime.timedelta(microseconds=9721))
        bus = Bus(logger)
        moves = [
            # heading zero
            [Rotation(0, 0, 0, 1), [0, 0, 0, 1]],

            # looking left 90 degrees
            # t265 turns around y axis +90
            # osgar turns round z axis +90
            [Rotation(0, 0.7071068, 0, 0.7071068), [0, 0, 0.7071068, 0.7071068]],

            # looking up 90 degrees
            # t265 turns around x axis +90
            # osgar turns round y axis -90
            [Rotation(0.7071068, 0, 0, 0.7071068), [0, -0.7071068, 0, 0.7071068]]
        ]
        c = RealSense(bus=bus.handle('rs'), config={})
        tester = bus.handle('tester')
        bus.connect('rs.orientation', 'tester.orientation')
        for input, output in moves:
            frame = MagicMock()
            pose_frame = frame.as_pose_frame.return_value
            pose_frame.get_pose_data.return_value = Pose(
                Acceleration(0, 0, 0),
                AngularAcceleration(0, 0, 0),
                AngularVelocity(0, 0, 0),
                0,
                input,
                0,
                Translation(0, 0, 0),
                Velocity(0, 0, 0))
            frame.get_timestamp.return_value = 0
            frame.get_frame_number.return_value = 0
            c.pose_callback(frame)
            dt, channel, orientation = tester.listen()
            self.assertEqual(channel, 'orientation')
            self.assertEqual(orientation, output)

    def test_pose2d_heading(self):
        logger = MagicMock()
        logger.write = MagicMock(return_value=datetime.timedelta(microseconds=9721))
        bus = Bus(logger)
        moves = [
            [Rotation(0, 0, 0, 1), [0, 0, 0]], # heading zero
            [Rotation(0, 0.7071068, 0, 0.7071068), [0, 0, 90*100]],   # facing left
            [Rotation(0, -0.7071068, 0, 0.7071068), [0, 0, -90*100]], # facing right
            [Rotation(0, -0.9999619, 0, 0.0087265), [0, 0, -179*100]],  # facing backwards
        ]
        c = RealSense(bus=bus.handle('rs'), config={})
        tester = bus.handle('tester')
        bus.connect('rs.pose2d', 'tester.pose2d')
        for input, output in moves:
            frame = MagicMock()
            pose_frame = frame.as_pose_frame.return_value
            pose_frame.get_pose_data.return_value = Pose(
                Acceleration(0, 0, 0),
                AngularAcceleration(0, 0, 0),
                AngularVelocity(0, 0, 0),
                0,
                input,
                0,
                Translation(0, 0, 0),
                Velocity(0, 0, 0))
            frame.get_timestamp.return_value = 0
            frame.get_frame_number.return_value = 0
            c.pose_callback(frame)
            dt, channel, pose2d = tester.listen()
            self.assertEqual(channel, 'pose2d')
            self.assertEqual(pose2d, output)

    def test_pose3d(self):
        logger = MagicMock()
        logger.write = MagicMock(return_value=datetime.timedelta(microseconds=9721))
        bus = Bus(logger)
        moves = [
            # heading zero
            [[Translation(0, 0, 0), Rotation(0, 0, 0, 1)], [[0, 0, 0], [0, 0, 0, 1]]],

            # looking left 90 degrees, moving front 1m
            # t265 turns around y axis +90, moving z -1
            # osgar turns round z axis +90, moving x 1
            [[Translation(0, 0, -1), Rotation(0, 0.7071068, 0, 0.7071068)], [[1, 0, 0], [0, 0, 0.7071068, 0.7071068]]],

            # looking up 90 degrees, moving up 1m
            # t265 turns around x axis +90, moving y 1
            # osgar turns round y axis -90, moving z 1
            [[Translation(0, 1, 0), Rotation(0.7071068, 0, 0, 0.7071068)], [[0, 0, 1], [0, -0.7071068, 0, 0.7071068]]]
        ]
        c = RealSense(bus=bus.handle('rs'), config={})
        tester = bus.handle('tester')
        bus.connect('rs.pose3d', 'tester.pose3d')
        for input, output in moves:
            frame = MagicMock()
            pose_frame = frame.as_pose_frame.return_value
            pose_frame.get_pose_data.return_value = Pose(
                Acceleration(0, 0, 0),
                AngularAcceleration(0, 0, 0),
                AngularVelocity(0, 0, 0),
                0,
                input[1],
                0,
                input[0],
                Velocity(0, 0, 0))
            frame.get_timestamp.return_value = 0
            frame.get_frame_number.return_value = 0
            c.pose_callback(frame)
            dt, channel, pose3d = tester.listen()
            self.assertEqual(channel, 'pose3d')
            self.assertEqual(pose3d, output)

    def test_depth(self):
        logger = MagicMock()
        logger.write = MagicMock(return_value=datetime.timedelta(microseconds=9721))
        bus = Bus(logger)
        c = RealSense(bus=bus.handle('rs'), config={})
        tester = bus.handle('tester')
        bus.connect('rs.depth', 'tester.depth')
        bus.connect('rs.color', 'tester.color')
        frameset = MagicMock()
        frameset.is_frameset.return_value = True
        frame = frameset.as_frameset.return_value.get_depth_frame.return_value
        frame.get_timestamp.return_value = 0
        frame.get_frame_number.return_value = 0
        frame.is_depth_frame.return_value = True
        frame.as_depth_frame.return_value.get_data.return_value = [1,2]

        color_frame = frameset.as_frameset.return_value.get_color_frame.return_value
        color_frame.get_timestamp.return_value = 0
        color_frame.get_frame_number.return_value = 0
        color_frame.is_frame.return_value = True
        color_frame.as_frame.return_value.get_data.return_value = np.asarray([[0, 100, 255]], dtype=np.uint8)

        c.depth_callback(frameset)
        dt, channel_1, depth = tester.listen()
        dt, channel_2, color = tester.listen()
        depth_expected = np.asanyarray([1,2])
        color_expected = np.asanyarray([[0, 100, 255]], dtype=np.uint8)
        self.assertEqual(channel_1, 'depth')
        self.assertEqual(channel_2, 'color')
        color = cv2.imdecode(np.fromstring(color, dtype=np.uint8), 0)
        self.assertEqual(depth.shape, depth_expected.shape)
        self.assertEqual(depth.dtype, depth_expected.dtype)
        self.assertTrue(np.array_equal(depth, depth_expected))
        self.assertEqual(color.shape, color_expected.shape)
#        self.assertTrue(np.array_equal(color, color_expected))

# vim: expandtab sw=4 ts=4
