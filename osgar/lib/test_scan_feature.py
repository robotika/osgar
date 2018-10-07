import unittest

from osgar.lib.scan_feature import extract_features, scan_split


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

# vim: expandtab sw=4 ts=4
