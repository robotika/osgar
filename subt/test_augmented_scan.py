import unittest
from unittest.mock import MagicMock

from subt.augmented_scan import AugmentedScan, compute_scan360
from osgar.lib import quaternion


class AugmentedScanTest(unittest.TestCase):

    def test_usage(self):
        bus = MagicMock()
        augmenter = AugmentedScan(bus=bus, config={})
        scan = [5000] * 270
        augmenter.on_scan(scan)
        bus.publish.assert_called()

    def test_barrier(self):
        barrier = [10, 20, 0]  # xyz
        scan = compute_scan360(barrier, radius=1, limit=10)
        self.assertEqual(min(scan), 0xFFFF)

        scan = compute_scan360(barrier, radius=1, limit=30)
        self.assertEqual(min(scan), 21360)

    def test_update(self):
        bus = MagicMock()
        augmenter = AugmentedScan(bus=bus, config={})
        augmenter.barrier = [[10, 5, 0], 1]
        augmenter.on_pose3d([[8, 5, 0], quaternion.identity()])
        scan = [5000] * 270
        augmenter.on_scan(scan)
        scan2 = [1000] * 31 + [5000] * 239
        bus.publish.assert_called_with('scan', scan2)

# vim: expandtab sw=4 ts=4
