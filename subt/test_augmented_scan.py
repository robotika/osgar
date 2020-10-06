import unittest
from unittest.mock import MagicMock

from subt.augmented_scan import AugmentedScan
from osgar.lib import quaternion


class AugmentedScanTest(unittest.TestCase):

    def test_usage(self):
        bus = MagicMock()
        bread = AugmentedScan(bus=bus, config={})
        scan = [5000] * 270
        bread.on_scan(scan)
        bus.publish.assert_called()

# vim: expandtab sw=4 ts=4
