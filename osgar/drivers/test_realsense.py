import unittest
from unittest.mock import patch, MagicMock
import datetime
import time
from collections import namedtuple

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
    def test_spin_once(self):
        logger = MagicMock()
        logger.write = MagicMock(return_value=datetime.timedelta(microseconds=9721))
        bus = Bus(logger)
        with patch('osgar.drivers.realsense.rs') as mock:
            instance = mock.return_value
            c = RealSense(bus=bus.handle('rs'), config={})
            tester = bus.handle('tester')
            tester.register('tick')
            bus.connect('tester.tick', 'rs.trigger')
            c.start()
            time.sleep(0.1)
            frames = c.pose_pipeline.wait_for_frames.return_value
            pose_frame = frames.get_pose_frame.return_value
            pose_frame.get_pose_data.return_value = Pose(
                Acceleration(0, 0, 0),
                AngularAcceleration(0, 0, 0),
                AngularVelocity(0, 0, 0),
                0,
                Rotation(0, 0, 0, 1),
                0,
                Translation(0, 0, 0),
                Velocity(0, 0, 0))
            pose_frame.get_timestamp.return_value = 0
            pose_frame.get_frame_number.return_value = 0
            tester.publish('tick', None)
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
        with patch('osgar.drivers.realsense.rs') as mock:
            instance = mock.return_value
            c = RealSense(bus=bus.handle('rs'), config={})
            tester = bus.handle('tester')
            tester.register('tick')
            bus.connect('tester.tick', 'rs.trigger')
            bus.connect('rs.pose2d', 'tester.pose2d')
            c.start()
            time.sleep(0.1)
            frames = c.pose_pipeline.wait_for_frames.return_value
            pose_frame = frames.get_pose_frame.return_value
            for input, output in moves:
                pose_frame.get_pose_data.return_value = Pose(
                    Acceleration(0, 0, 0),
                    AngularAcceleration(0, 0, 0),
                    AngularVelocity(0, 0, 0),
                    0,
                    Rotation(0, 0, 0, 1),
                    0,
                    input,
                    Velocity(0, 0, 0))
                pose_frame.get_timestamp.return_value = 0
                pose_frame.get_frame_number.return_value = 0
                tester.publish('tick', None)
                dt, channel, pose2d = tester.listen()
                self.assertEqual(channel, 'pose2d')
                self.assertEqual(pose2d, output)
            c.request_stop()
            c.join()
            return

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
        with patch('osgar.drivers.realsense.rs') as mock:
            instance = mock.return_value
            c = RealSense(bus=bus.handle('rs'), config={})
            tester = bus.handle('tester')
            tester.register('tick')
            bus.connect('tester.tick', 'rs.trigger')
            bus.connect('rs.orientation', 'tester.orientation')
            c.start()
            time.sleep(0.1)
            frames = c.pose_pipeline.wait_for_frames.return_value
            pose_frame = frames.get_pose_frame.return_value
            for input, output in moves:
                pose_frame.get_pose_data.return_value = Pose(
                    Acceleration(0, 0, 0),
                    AngularAcceleration(0, 0, 0),
                    AngularVelocity(0, 0, 0),
                    0,
                    input,
                    0,
                    Translation(0, 0, 0),
                    Velocity(0, 0, 0))
                pose_frame.get_timestamp.return_value = 0
                pose_frame.get_frame_number.return_value = 0
                tester.publish('tick', None)
                dt, channel, orientation = tester.listen()
                self.assertEqual(channel, 'orientation')
                self.assertEqual(orientation, output)
            c.request_stop()
            c.join()
            return

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
        with patch('osgar.drivers.realsense.rs') as mock:
            instance = mock.return_value
            c = RealSense(bus=bus.handle('rs'), config={})
            tester = bus.handle('tester')
            tester.register('tick')
            bus.connect('tester.tick', 'rs.trigger')
            bus.connect('rs.pose2d', 'tester.pose2d')
            c.start()
            time.sleep(0.1)
            frames = c.pose_pipeline.wait_for_frames.return_value
            pose_frame = frames.get_pose_frame.return_value
            for input, output in moves:
                pose_frame.get_pose_data.return_value = Pose(
                    Acceleration(0, 0, 0),
                    AngularAcceleration(0, 0, 0),
                    AngularVelocity(0, 0, 0),
                    0,
                    input,
                    0,
                    Translation(0, 0, 0),
                    Velocity(0, 0, 0))
                pose_frame.get_timestamp.return_value = 0
                pose_frame.get_frame_number.return_value = 0
                tester.publish('tick', None)
                dt, channel, pose2d = tester.listen()
                self.assertEqual(channel, 'pose2d')
                self.assertEqual(pose2d, output)
            c.request_stop()
            c.join()
            return

    def test_pose3d(self):
        logger = MagicMock()
        logger.write = MagicMock(return_value=datetime.timedelta(microseconds=9721))
        bus = Bus(logger)
        item = namedtuple('item', ['input', 'output'])
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
        with patch('osgar.drivers.realsense.rs') as mock:
            instance = mock.return_value
            c = RealSense(bus=bus.handle('rs'), config={})
            tester = bus.handle('tester')
            tester.register('tick')
            bus.connect('tester.tick', 'rs.trigger')
            bus.connect('rs.pose3d', 'tester.pose3d')
            c.start()
            time.sleep(0.1)
            frames = c.pose_pipeline.wait_for_frames.return_value
            pose_frame = frames.get_pose_frame.return_value
            for input, output in moves:
                pose_frame.get_pose_data.return_value = Pose(
                    Acceleration(0, 0, 0),
                    AngularAcceleration(0, 0, 0),
                    AngularVelocity(0, 0, 0),
                    0,
                    input[1],
                    0,
                    input[0],
                    Velocity(0, 0, 0))
                pose_frame.get_timestamp.return_value = 0
                pose_frame.get_frame_number.return_value = 0
                tester.publish('tick', None)
                dt, channel, pose3d = tester.listen()
                self.assertEqual(channel, 'pose3d')
                self.assertEqual(pose3d, output)
            c.request_stop()
            c.join()
            return


# vim: expandtab sw=4 ts=4
