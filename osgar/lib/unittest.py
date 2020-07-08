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
        np.testing.assert_almost_equal(n1, n2, msg)

    def assertPose3dEqual(self, p1, p2):
        self.assertEqual(len(p1), 2, msg='First Pose3d needs xyz and quaternion')
        self.assertEqual(len(p2), 2, msg='Second Pose3d needs xyz and quaternion')
        self.assertXYZEqual(p1[0], p2[0])
        self.assertQuaternionEqual(p1[1], p2[1])

    def assertQuaternionEqual(self, q1, q2):
        self.assertAlmostEqual(q1[0], q2[0], places=6, msg="qx differs")
        self.assertAlmostEqual(q1[1], q2[1], places=6, msg="qy differs")
        self.assertAlmostEqual(q1[2], q2[2], places=6, msg="qz differs")
        self.assertAlmostEqual(q1[3], q2[3], places=6, msg="qw differs")

    def assertXYZEqual(self, xyz1, xyz2):
        self.assertAlmostEqual(xyz1[0], xyz2[0], places=6, msg="x differs")
        self.assertAlmostEqual(xyz1[1], xyz2[1], places=6, msg="y differs")
        self.assertAlmostEqual(xyz1[2], xyz2[2], places=6, msg="z differs")
