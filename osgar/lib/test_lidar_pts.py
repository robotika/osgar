import unittest

from osgar.lib.lidar_pts import equal_scans


class LidarPtsTest(unittest.TestCase):

    def test_equal_scans(self):
        self.assertTrue(equal_scans([1, 2, 3], [1, 2, 3]))
        self.assertFalse(equal_scans([1, 2, 3], [1, 4, 3], tollerance=1))


# vim: expandtab sw=4 ts=4
