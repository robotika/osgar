import unittest
from unittest.mock import MagicMock, call

import cv2

from follow_apriltag import AprilTag


class AprilTagTest(unittest.TestCase):
    def test_detect_april_tags(self):
        config = {}
        bus = MagicMock()
        node = AprilTag(config, bus)

        image = cv2.imread("april-tags.jpg", 0)
        tags = node.detect_april_tags(image)
        self.assertEqual(tags[0], [3, 4, 1, 2])
        self.assertEqual(len(tags[1][0]), 4)
        self.assertEqual(tags[1][0][0], [664, 684])

    def test_corners_to_dist(self):
        config = {}
        bus = MagicMock()
        node = AprilTag(config, bus)

        tags = [[5], [[[1392, 879], [1340, 878], [1338, 827], [1389, 827]]]]  # 0:00:11.570785
        self.assertAlmostEqual(node.corners_to_dist(tags[1][0]), 1.249184833037426)
        tags = [[5], [[[1903, 1001], [1795, 997], [1789, 898], [1891, 901]]]]  # 0:00:12.870132
        self.assertAlmostEqual(node.corners_to_dist(tags[1][0]), 0.6288112907630552)

    def test_corners_to_angle(self):
        config = {}
        bus = MagicMock()
        node = AprilTag(config, bus)

        tags = [[5], [[[1392, 879], [1340, 878], [1338, 827], [1389, 827]]]]  # 0:00:11.570785
        self.assertAlmostEqual(node.corners_to_angle(tags[1][0]), -0.2538704115488783)


if __name__ == '__main__':
    unittest.main()
