import math
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
        mat = transform(pairs)
        self.assertAlmostEqual(dx, -mat[0][2])
        self.assertAlmostEqual(dy, -mat[1][2])
        np.testing.assert_almost_equal(mat[:2, :2], np.identity(2))

    def test_rotation(self):
        pairs = [
            ((0, 0), (0, 0)),
            ((10, 0), (0, 10)),
            ((10, 10), (-10, 10))
        ]
        mat = transform(pairs)
        np.testing.assert_almost_equal(mat[:2, :2], np.array([[0, 1], [-1, 0]]))

        dx = 0.1
        dy = -0.3
        shifted_pairs = []
        for (x1, y1), (x2, y2) in pairs:
            shifted_pairs.append(((x1, y1), (x2+dx, y2+dy)))
        mat = transform(shifted_pairs)
        np.testing.assert_almost_equal(mat[:2, :2], np.array([[0, 1], [-1, 0]]))
        self.assertAlmostEqual(dx, mat[1][2])  # after rotation
        self.assertAlmostEqual(dy, -mat[0][2])

    def test_icp(self):
        dx = 0.1
        dy = -0.3
        scan1 = [(0, 0), (10, 0), (10, 10)]
        scan2 = []
        for x, y in scan1:
            scan2.append((x+dx, y+dy))
        mat = my_icp(scan1, scan2, num_iter=3)
        scan2corrected = np.array([(x, y) for x, y, one in np.matmul(mat, np.array([[x, y, 1] for x, y, in scan2]).T).T])
        np.testing.assert_almost_equal(np.array(scan1), scan2corrected)

    def test_rotation_with_translation(self):
        angle = math.radians(10)
        dx = 0.1
        dy = -0.3
        scan2 = [(0, 0), (10, 0), (10, 10), (2, 13)]

        mat = np.array([
            [math.cos(angle),  math.sin(angle), dx],
            [-math.sin(angle), math.cos(angle), dy],
            [0,                0,               1]
        ])
        scan1 = (mat @ np.array([[x, y, 1] for x, y in scan2]).T).T[:, :2].tolist()
        mat2 = my_icp(scan1, scan2, num_iter=3)
        np.testing.assert_almost_equal(mat, mat2)
