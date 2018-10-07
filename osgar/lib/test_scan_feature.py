import unittest

from osgar.lib.scan_feature import extract_features


class ScanFeatureTest(unittest.TestCase):

    def test_extract_feature(self):
        scan = [0]*811
        self.assertIsNone(extract_features(scan))

# vim: expandtab sw=4 ts=4
