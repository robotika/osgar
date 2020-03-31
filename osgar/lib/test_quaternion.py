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


# vim: expandtab sw=4 ts=4
