import unittest
import math

import numpy as np

from subt.depth2scan import vertical_step, monotize, find_step


class Depth2ScanTest(unittest.TestCase):

    def test_vertical_step(self):
        depth = np.zeros((360, 640), dtype=np.uint16)
        self.assertIsNone(vertical_step(depth))

        depth[:, 320] = np.arange(0, 4*360, 4)
        #self.assertIsNone(vertical_step(depth))

        depth = np.zeros((360, 640), dtype=np.uint16)
        depth[:,:] = 200
        for i in range(180, 250):
            depth[i, 320] = 400
        self.assertEqual(vertical_step(depth), 200)  # TODO better example

    def test_monotize(self):
        a = np.zeros(10, dtype=np.int32)
        np.testing.assert_equal(monotize(a), a)
        a[7] = 13
        self.assertEqual(monotize(a)[-1], 13)

    def test_find_step(self):
        self.assertIsNone(find_step(np.zeros(250, dtype=np.int32)))
        
        arr = np.zeros(250, dtype=np.int32)
        arr[100:] = 200  # wooden crate 20cm high
        self.assertEqual(find_step(arr), 100)

# vim: expandtab sw=4 ts=4

