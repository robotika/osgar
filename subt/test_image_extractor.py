import unittest
from unittest.mock import MagicMock

from subt.image_extractor import ImageExtractor


class ImageExtractorTest(unittest.TestCase):

    def test_min_dist(self):
        bus = MagicMock()
        logimage = ImageExtractor(bus=bus, config={})

        robot_pose = None
        camera_pose = None
        rgb_compressed = b'dummy image'
        depth_compressed = None

        data = robot_pose, camera_pose, rgb_compressed, depth_compressed
        logimage.on_rgbd(data)

        bus.publish.assert_called_with('image', b'dummy image')


# vim: expandtab sw=4 ts=4
