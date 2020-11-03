import unittest
from unittest.mock import MagicMock
import math

import numpy as np

from subt.points2scan import PointsToScan


class PointsToScanTest(unittest.TestCase):

    def test_numpy(self):
        bus = MagicMock()
        conv = PointsToScan(bus=bus, config={})

        orig_data = np.zeros((16, 10000, 3), dtype=np.float32)
        data = orig_data[:, ::10, :]  # downsample to everh 10th
        conv.on_points(data)
        bus.publish.assert_called()

# vim: expandtab sw=4 ts=4

