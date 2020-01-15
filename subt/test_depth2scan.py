import unittest
import math

import numpy as np

from subt.depth2scan import vertical_step


class Depth2ScanTest(unittest.TestCase):

    def test_vertical_step(self):
        depth = np.zeros((360, 640), dtype=np.uint16)
        self.assertIsNone(vertical_step(depth))

        depth[:, 320] = np.arange(0, 4*360, 4)
        self.assertIsNone(vertical_step(depth))

        for i in range(100, 120):
            depth[i, 320] = 400
        self.assertEqual(vertical_step(depth), 400)

# vim: expandtab sw=4 ts=4

