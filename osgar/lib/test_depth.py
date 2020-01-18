import unittest

import numpy as np

from osgar.lib.depth import danger2dist


class DepthTest(unittest.TestCase):

    def test_danger2dist(self):
        danger = np.zeros((360, 640), dtype=bool)
        danger[100][100] = True
        danger[200][630] = True
        dist = danger2dist(danger)
        self.assertEqual(len(dist), 640)
        self.assertEqual(dist[100], 100)
        self.assertEqual(dist[630], 200)

# vim: expandtab sw=4 ts=4
