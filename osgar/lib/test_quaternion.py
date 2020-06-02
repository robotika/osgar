import math
import unittest

from . import quaternion


class QuaternionTest(unittest.TestCase):

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
        self.assertAlmostEqual(expected[0], actual[0], places=6)
        self.assertAlmostEqual(expected[1], actual[1], places=6)
        self.assertAlmostEqual(expected[2], actual[2], places=6)

        actual = quaternion.rotate_vector(expected, q)
        expected = [-1, 0, 0]
        self.assertAlmostEqual(expected[0], actual[0], places=6)
        self.assertAlmostEqual(expected[1], actual[1], places=6)
        self.assertAlmostEqual(expected[2], actual[2], places=6)

    def test_from_axis_angle(self):
        actual = quaternion.from_axis_angle((0,0,1), math.radians(90))
        expected = [0, 0, 0.7071068, 0.7071068]
        self.assertAlmostEqual(expected[0], actual[0], places=6)
        self.assertAlmostEqual(expected[1], actual[1], places=6)
        self.assertAlmostEqual(expected[2], actual[2], places=6)
        self.assertAlmostEqual(expected[3], actual[3], places=6)

    def test_asin_domain(self):
        q = (0.02089575225593203, -0.6597655110187111, 0.381406681115162, -0.7726979260818011)
        angles = quaternion.euler_zyx(q)  # should not raise ValueError: math domain error
        self.assertAlmostEqual(angles[0], -1.5436762)
        self.assertAlmostEqual(angles[1], 1.0194396)
        self.assertAlmostEqual(angles[2], -1.0505728)

# vim: expandtab sw=4 ts=4
