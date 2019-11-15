import unittest

from . import quaternion


class QuaternionTest(unittest.TestCase):

    def test_multiply(self):
        qa = [1, 0, 0, 0]
        qb = [0, 1, 0, 0]
        qc = quaternion.multiply(qa, qb)
        self.assertEqual(qc, qb)  # identity

        qd = quaternion.multiply(qb, qb)
        self.assertEqual(qd, [-1, 0, 0, 0])  # twice rotation by 90deg

        qdd = quaternion.multiply(qd, qd)
        self.assertEqual(qdd, qa)  # identity

# vim: expandtab sw=4 ts=4
