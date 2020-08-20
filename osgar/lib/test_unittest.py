from .unittest import TestCase

import numpy as np


class Test(TestCase):

    def test_numpy(self):
        template = [1, 2, 3]
        a = np.asarray(template)
        b = np.asarray(template)
        self.assertEqual(a, b)
        self.assertNumpyEqual(a, b)

        with self.assertRaises(AssertionError):
            self.assertNumpyEqual(a, [])
        with self.assertRaises(AssertionError):
            self.assertNumpyEqual(a, template)

    def test_pose3d(self):
        a = [[0,0,0], [0,0,0,1]]
        with self.assertRaises(AssertionError):
            self.assertPose3dEqual([], [])

        self.assertPose3dEqual(a, a)

    def test_list(self):
        self.assertListAlmostEqual([1], [1])
        self.assertListAlmostEqual([1, 2], [1, 2])
        self.assertListAlmostEqual([1.1, 2], [1.1, 2])
        self.assertListAlmostEqual([1.10000001, 2], [1.1, 2])

        with self.assertRaises(AssertionError):
            self.assertListAlmostEqual([1.1000001, 2], [1.1, 2])
        with self.assertRaises(AssertionError):
            self.assertListAlmostEqual([1, 2], [1, 3])
        with self.assertRaises(AssertionError):
            self.assertListAlmostEqual([1, 2], [1, 2, 3])
