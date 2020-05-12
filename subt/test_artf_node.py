import unittest

import numpy as np

from subt.artf_node import result2report


class ArtifactDetectorDNNTest(unittest.TestCase):

    def test_result2report(self):
        result = [('backpack', [(60, 180, 0.9785775), (72, 180, 0.9795098), (60, 184, 0.97716093), (72, 184, 0.9782014)])]
        row = [5000]*640
        depth = np.array([row]*360, dtype=np.uint16)
        self.assertEqual(result2report(result, depth), ['TYPE_BACKPACK', 2754, 5000])

        row = list(range(640))
        depth = np.array([row]*360, dtype=np.uint16)
        self.assertEqual(result2report(result, depth), ['TYPE_BACKPACK', 2754, 66])

        result2 = [('rope', [(400, 180, 0.9785775)])]
        self.assertEqual(result2report(result2, depth), ['TYPE_ROPE', -868, 400])

    def test_out_of_range(self):
        result = [('backpack', [(60, 180, 0.9785775), (72, 180, 0.9795098), (60, 184, 0.97716093), (72, 184, 0.9782014)])]
        row = [0xFFFF]*640
        depth = np.array([row]*360, dtype=np.uint16)
        self.assertEqual(result2report(result, depth), None)

# vim: expandtab sw=4 ts=4
