import unittest
from unittest.mock import MagicMock
import os

import cv2

from subt.octomap import Octomap, draw_map


class OctomapTest(unittest.TestCase):

    def test_map(self):
        with open(os.path.join(os.path.dirname(__file__), 'test_data', 'freyja-octomap.bin'), 'rb') as f:
            data = f.read()
        self.assertEqual(len(data), 57278)
        img = draw_map(data)
        cv2.imwrite('octo.jpg', img)

    def Xtest_numpy(self):
        bus = MagicMock()
        conv = PointsToScan(bus=bus, config={})

        orig_data = np.zeros((16, 10000, 3), dtype=np.float32)
        data = orig_data[:, ::10, :]  # downsample to everh 10th
        conv.on_points(data)
        bus.publish.assert_called()

# vim: expandtab sw=4 ts=4

