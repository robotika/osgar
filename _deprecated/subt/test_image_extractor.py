import math
import unittest
from unittest.mock import MagicMock

from osgar.lib import quaternion

from subt.image_extractor import ImageExtractor


class ImageExtractorTest(unittest.TestCase):

    def test_usage(self):
        bus = MagicMock()
        logimage = ImageExtractor(bus=bus, config={})

        robot_pose = None
        camera_pose = None
        rgb_compressed = b'dummy image'
        depth_compressed = None

        data = robot_pose, camera_pose, rgb_compressed, depth_compressed
        logimage.on_rgbd(data)

        bus.publish.assert_called_with('image', b'dummy image')

    def test_min_dist(self):
        bus = MagicMock()
        logimage = ImageExtractor(bus=bus, config={})

        robot_pose = [[0, 0, 0], [0, 0, 0, 1]]
        camera_pose = None
        rgb_compressed = b'dummy image'
        depth_compressed = None

        bus.reset_mock()
        for i in range(10):
            data = robot_pose, camera_pose, rgb_compressed, depth_compressed
            logimage.on_rgbd(data)

        self.assertEqual(len(bus.method_calls), 1)

    def test_min_dist(self):
        bus = MagicMock()
        logimage = ImageExtractor(bus=bus, config={})

        robot_pose = [[0, 0, 0], [0, 0, 0, 1]]
        angle = quaternion.euler_to_quaternion(yaw=math.radians(20), roll=0, pitch=0)
        camera_pose = None
        rgb_compressed = b'dummy image'
        depth_compressed = None

        bus.reset_mock()
        for i in range(10):
            data = robot_pose, camera_pose, rgb_compressed, depth_compressed
            logimage.on_rgbd(data)
            robot_pose = [robot_pose[0], quaternion.multiply(robot_pose[1], angle)]

        self.assertEqual(len(bus.method_calls), 10)


# vim: expandtab sw=4 ts=4
