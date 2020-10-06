import unittest
from unittest.mock import MagicMock

from subt.augmented_scan import AugmentedScan, compute_scan360
from osgar.lib import quaternion


class AugmentedScanTest(unittest.TestCase):

    def test_usage(self):
        bus = MagicMock()
        bread = AugmentedScan(bus=bus, config={})
        scan = [5000] * 270
        bread.on_scan(scan)
        bus.publish.assert_called()

    def test_barrier(self):
        barrier = [[10, 20, 0]]  # xyz
        scan = compute_scan360(barrier, limit=10)
        self.assertEqual(max(scan), 0)

        scan = compute_scan360(barrier, limit=30)
        self.assertEqual(max(scan), 22360)

    def test_update(self):
        bus = MagicMock()
        bread = AugmentedScan(bus=bus, config={})
        bread.barrier = [[10, 5, 0]]
        bread.on_pose3d([[8, 5, 0], quaternion.identity()])
        scan = [5000] * 270
        bread.on_scan(scan)
        scan2 = [5000] * 270
        scan2[0] = 2000
        bus.publish.assert_called_with('scan', scan2)

# vim: expandtab sw=4 ts=4
