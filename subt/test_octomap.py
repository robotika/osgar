import unittest
from unittest.mock import MagicMock
import os

import cv2

from subt.octomap import Octomap, data2maplevel


class OctomapTest(unittest.TestCase):

    def test_map(self):
        with open(os.path.join(os.path.dirname(__file__), 'test_data', 'freyja-octomap.bin'), 'rb') as f:
            data = f.read()
        self.assertEqual(len(data), 57278)
        img = data2maplevel(data, level=2)
        cv2.imwrite('octo.png', img)

# vim: expandtab sw=4 ts=4

