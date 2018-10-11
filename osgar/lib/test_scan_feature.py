import unittest

from osgar.lib.scan_feature import (extract_features, scan_split,
                                    is_box_center, find_transporter)


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

    def test_find_transporter(self):
        wall = [2245, 2262, 2291, 2348, 2424, 2493, 2198, 1975, 1790, 1668, 1552, 1480, 1417,
                1370, 1339, 1310, 1295, 1226, 1288, 1306, 1323, 1111, 1131, 1205, 1533, 1615, 1742,
                1801, 1798, 2833, 2204, 2172, 2123, 2820, 3439, 2460]
        self.assertEqual(find_transporter(wall), None)

        trans = [3031, 3032, 3071, 3124, 3206, 3314, 3137, 3126, 2985, 3216, 2094, 2122, 3392, 3571, 3724,
                3651, 3587, 3158, 3153, 3179, 3102, 3014, 2960, 2898, 2899, 2925, 2936, 2967, 3006, 3047, 3084,
                3164, 3299, 3427, 3573, 3643]
        self.assertEqual(find_transporter(trans), (10, 11))

# vim: expandtab sw=4 ts=4
