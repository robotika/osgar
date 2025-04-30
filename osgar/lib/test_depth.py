import unittest

import numpy as np

from osgar.lib.depth import depth2dist, depth_to_rgb_align


class DepthTest(unittest.TestCase):

    def Xtest_danger2dist(self):
        danger = np.zeros((360, 640), dtype=bool)
        danger[100][100] = True
        danger[200][630] = True
        dist = danger2dist(danger)
        self.assertEqual(len(dist), 640)
        self.assertEqual(dist[100], 100)
        self.assertEqual(dist[630], 200)

        values = danger[dist, np.arange(640)]
        self.assertTrue(values[100])
        self.assertTrue(values[630])
        self.assertFalse(values[300])

    def test_depth2rgb(self):
        T = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        intr_M = [[100, 0, 1], [0, 100, 1], [0, 0, 1]]
        depth = np.ones((3,3), dtype=int) * 2000
        depth2 = depth_to_rgb_align(depth, intr_M, intr_M, T, [3,3, 3])
        print(depth2)
        self.assertTrue(np.array_equal(depth, depth2))

        intr_rgb = [[200, 0, 2.5], [0, 200, 2.5], [0, 0, 1]]
        depth2 = depth_to_rgb_align(depth, intr_M, intr_rgb, T, [6, 6, 3])
        print(depth2)

# vim: expandtab sw=4 ts=4
