import unittest
from unittest.mock import MagicMock

import numpy as np

from subt.three2scan import ThreeRGBDToScan


class ThreeRGBDToScanTest(unittest.TestCase):

    def test_numpy(self):
        bus = MagicMock()
        conv = ThreeRGBDToScan(bus=bus, config={})

        data = np.zeros((16, 1800, 3), dtype=np.float32)
        conv.on_rgbd(data)
        bus.publish.assert_called()

# vim: expandtab sw=4 ts=4

