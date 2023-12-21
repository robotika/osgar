import unittest

import numpy as np

from osgar.lib.icp import transform, my_icp


class ICPTest(unittest.TestCase):
    def test_translation(self):
        dx = 0.1
        dy = -0.3
        pts = [(0, 0), (10, 0), (10, 10)]
        pairs = []
        for x, y in pts:
            pairs.append(((x, y), (x+dx, y+dy)))
        trans, rot = transform(pairs)
        self.assertAlmostEqual(dx, -trans[0])
        self.assertAlmostEqual(dy, -trans[1])
        np.testing.assert_almost_equal(rot, np.identity(2))

    def test_rotation(self):
        pairs = [
            ((0, 0), (0, 0)),
            ((10, 0), (0, 10)),
            ((10, 10), (-10, 10))
        ]
        trans, rot = transform(pairs)
        np.testing.assert_almost_equal(rot, np.array([[0, 1], [-1, 0]]))

        dx = 0.1
        dy = -0.3
        shifted_pairs = []
        for (x1, y1), (x2, y2) in pairs:
            shifted_pairs.append(((x1, y1), (x2+dx, y2+dy)))
        trans, rot = transform(shifted_pairs)
        np.testing.assert_almost_equal(rot, np.array([[0, 1], [-1, 0]]))
        self.assertAlmostEqual(dx, -trans[0])
        self.assertAlmostEqual(dy, -trans[1])

    def test_icp(self):
        dx = 0.1
        dy = -0.3
        scan1 = [(0, 0), (10, 0), (10, 10)]
        scan2 = []
        for x, y in scan1:
            scan2.append((x+dx, y+dy))
        my_icp(scan1, scan2)
        trans, rot = transform(pairs)
        self.assertAlmostEqual(dx, -trans[0])
        self.assertAlmostEqual(dy, -trans[1])
        np.testing.assert_almost_equal(rot, np.identity(2))
