import os
import unittest
from unittest.mock import MagicMock

import cv2

from moon.skyline import skyline


class SkylineTest(unittest.TestCase):
    def test_usage(self):
        img = cv2.imread(os.path.join(os.path.dirname(__file__), 'test_data', 'skyline.jpg'))
        self.assertEqual(img.shape, (480, 640, 3))

        s = skyline(img)
        self.assertEqual(len(s), 640)

# vim: expandtab sw=4 ts=4

