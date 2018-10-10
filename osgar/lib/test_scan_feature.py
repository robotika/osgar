import unittest

from osgar.lib.scan_feature import extract_features, scan_split, is_box_center


class ScanFeatureTest(unittest.TestCase):

    def Xtest_extract_feature(self):
        scan = [0]*811
        self.assertIsNone(extract_features(scan))

    def test_scan_split(self):
        scan = [1000]*180
        self.assertEqual(scan_split(scan, max_diff=15), [(0, 179)])
        box = [700]*30
        self.assertEqual(scan_split(scan+box+scan, max_diff=15), 
                [(0, 179), (180, 209), (210, 389)])

    def test_is_box_center(self):
        scan = [1000]*100
        self.assertFalse(is_box_center(50, scan))
        box = [700]*17*4
        self.assertTrue(is_box_center(100+2*17, scan+box+scan))

    def test_bug_division_by_zero(self):
        # corresponds to 0.32688 degree, which clips to 0 for 1/3 resolution
        scan = [17528]*811
        self.assertFalse(is_box_center(400, scan))

# vim: expandtab sw=4 ts=4
