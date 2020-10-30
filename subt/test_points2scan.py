import unittest
from unittest.mock import MagicMock
import math

import numpy as np

from subt.points2scan import PointsToScan


class PointsToScanTest(unittest.TestCase):

    def test_numpy(self):
        bus = MagicMock()
        conv = PointsToScan(bus=bus, config={})

        data = []
        conv.on_points(data)
        bus.publish.assert_called()

# vim: expandtab sw=4 ts=4

