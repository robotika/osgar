import unittest
from unittest.mock import MagicMock
import os

import cv2
import numpy as np

from subt.octomap import Octomap, data2maplevel, frontiers


class OctomapTest(unittest.TestCase):

    def test_map(self):
        with open(os.path.join(os.path.dirname(__file__), 'test_data', 'freyja-octomap.bin'), 'rb') as f:
            data = f.read()
        self.assertEqual(len(data), 57278)
        img = data2maplevel(data, level=2)
        cv2.imwrite('octo.png', img)

    def test_frontiers(self):
        # no frontiers in totally unknown world
        img = np.zeros((1024, 1024, 1), dtype=np.uint8)
        __, path = frontiers(img, start=(512, 512, 0))
        self.assertIsNone(path)

# vim: expandtab sw=4 ts=4

