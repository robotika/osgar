import math
from osgar.lib.unittest import TestCase

from . import quaternion


class QuaternionTest(TestCase):

    def test_multiply(self):
        qa = [0, 0, 0, 1]
        qb = [1, 0, 0, 0]
        qc = quaternion.multiply(qa, qb)
        self.assertEqual(qc, qb)  # identity

        qd = quaternion.multiply(qb, qb)
        self.assertEqual(qd, [0, 0, 0, -1])  # twice rotation by 90deg

        qdd = quaternion.multiply(qd, qd)
        self.assertEqual(qdd, qa)  # identity

    def test_rotate_z90(self):
        # rotate around Z axis by 90 deg (change heading)
        v = [1, 0, 0]
        q = [ 0, 0, 0.7071068, 0.7071068 ]
        actual = quaternion.rotate_vector(v, q)
        expected = [0, 1, 0]
        self.assertListAlmostEqual(expected, actual, delta=1e-6)

        actual = quaternion.rotate_vector(expected, q)
        expected = [-1, 0, 0]
        self.assertListAlmostEqual(expected, actual, delta=1e-6)

    def test_from_axis_angle(self):
        actual = quaternion.from_axis_angle((0,0,1), math.radians(90))
        expected = [0, 0, 0.7071068, 0.7071068]
        self.assertListAlmostEqual(expected, actual)

    def test_quaterion_internal_normalization(self):
        q = (0.02089575225593203, -0.6597655110187111, 0.381406681115162, -0.7726979260818011)
        angles = quaternion.euler_zyx(q)
        expected = [-1.5436762, 1.0194396, -1.0505728]
        self.assertListAlmostEqual(angles, expected)

    def test_rotation_matrix(self):
        from collections import namedtuple
        T = namedtuple('T', ('q', 'matrix'))
        tests = [
            T([ 0, 0, 0.7071068, 0.7071068 ], [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]),
            T([ 0, 0.7071068, 0, 0.7071068 ], [[0.0, 0.0, 1.0], [0.0,  1.0,  0.0], [-1.0, 0.0, 0.0]]),
            T([ 0.7071068, 0, 0, 0.7071068 ], [[1.0,  0.0,  0.0], [0.0,  0.0, -1.0], [0.0, 1.0, 0.0 ]]),
            T([ 0.5, 0.5, 0, 0.7071068 ], [[0.5, 0.5, 0.7071068], [0.5, 0.5, -0.7071068], [-0.7071068, 0.7071068, 0.0]])
        ]
        for i, t in enumerate(tests):
            with self.subTest(test=i):
                actual = quaternion.rotation_matrix(t.q)
                for row in range(len(actual)):
                    for col in range(len(actual[0])):
                        self.assertAlmostEqual(t.matrix[row][col], actual[row][col], places=6, msg=(row, col))

# vim: expandtab sw=4 ts=4
