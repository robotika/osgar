import unittest

import numpy as np

# if this variable is set, stack trace on test failure from this module is not in the output
__unittest = True

class TestCase(unittest.TestCase):

    def __init__(self, methodName='runTest'):
        super().__init__(methodName)
        self.addTypeEqualityFunc(np.ndarray, 'assertNumpyEqual')

    def assertNumpyEqual(self, n1, n2, msg=None):
        self.assertIsInstance(n1, np.ndarray, 'First argument is not a numpy.ndarray')
        self.assertIsInstance(n2, np.ndarray, 'Second argument is not a numpy.ndarray')
        np.testing.assert_almost_equal(n1, n2, err_msg=msg)

    def assertPose3dEqual(self, p1, p2):
        self.assertEqual(len(p1), 2, msg='First Pose3d needs xyz and quaternion')
        self.assertEqual(len(p2), 2, msg='Second Pose3d needs xyz and quaternion')
        self.assertEqual(len(p1[0]), 3, msg='First pose3d xyz needs 3 coordinates')
        self.assertEqual(len(p1[1]), 4, msg='First pose3d quaternion needs 4 coordinates')
        self.assertEqual(len(p2[0]), 3, msg='Second pose3d xyz needs 3 coordinates')
        self.assertEqual(len(p2[1]), 4, msg='Second pose3d quaternion needs 4 coordinates')
        self.assertListAlmostEqual(p1[0], p2[0])
        self.assertListAlmostEqual(p1[1], p2[1])

    def assertListAlmostEqual(self, l1, l2, places=None, delta=None):
        self.assertIsInstance(l1, list, 'First argument is not a list')
        self.assertIsInstance(l2, list, 'Second argument is not a list')
        self.assertEqual(len(l1), len(l2), "")
        for i, (a, b) in enumerate(zip(l1, l2)):
            self.assertAlmostEqual(a, b, places=places, msg=f"\nFirst differing element index: {i}", delta=delta)
