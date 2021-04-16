import unittest

import numpy as np

from osgar.lib.pplanner import find_path


class PathPlannerTest(unittest.TestCase):

    def test_usage(self):
        img = np.zeros((10, 10), dtype=bool)
        img[5,:] = True
        img[:, 0] = True
        self.assertTrue(img[0][0])
        self.assertTrue(img[5][5])
        self.assertFalse(img[3][3])

        self.assertIsNone(find_path(img, (3, 3), [(3, 3)]))

        path = find_path(img, (0, 0), [(5, 5)])
        self.assertEqual(path[0], (0, 0, 0))
        self.assertEqual(path[-1], (5, 5, 0))
        self.assertEqual(len(path), 5 + 1 + 5)

    def test_multiple_goals(self):
        img = np.zeros((10, 10), dtype=bool)
        img[5,:] = True
        img[:, 0] = True

        path = find_path(img, (0, 0), [(3, 3), (5, 5)])
        self.assertEqual(path[-1], (5, 5, 0))
        self.assertEqual(len(path), 5 + 1 + 5)

    def test_start_out_of_range(self):
        img = np.zeros((10, 10), dtype=bool)
        path = find_path(img, (200, 0), [(3, 3), (5, 5)])
        self.assertIsNone(path)

        img = np.zeros((10, 10, 5), dtype=bool)
        path = find_path(img, (0, 0, 13), [(3, 3, 0)])
        self.assertIsNone(path)

    def test_3d_planner(self):
        mimg = np.zeros((10, 10, 5), dtype=bool)
        mimg[5, :, 3] = True
        mimg[:, 7, 2] = True
        path = find_path(mimg, (0, 5, 3), [(7, 9, 2)])
        self.assertIsNotNone(path)

# vim: expandtab sw=4 ts=4

