import math
import random
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
            T([ 0.5, 0.5, 0, 0.7071068 ], [[0.5, 0.5, 0.7071068], [0.5, 0.5, -0.7071068], [-0.7071068, 0.7071068, 0.0]]),
            T([ 0, 0, 1, 0 ], [[-1.0, -0.0,  0.0], [0.0000000, -1.0,  0.0], [0.0,  0.0,  1.0]]),
        ]
        for i, t in enumerate(tests):
            with self.subTest(test=i, kind="to matrix"):
                actual = quaternion.rotation_matrix(t.q)
                for row in range(len(actual)):
                    for col in range(len(actual[0])):
                        self.assertAlmostEqual(t.matrix[row][col], actual[row][col], places=6, msg=(row, col))

        for i, t in enumerate(tests):
            with self.subTest(test=i, kind="from matrix"):
                q = quaternion.from_rotation_matrix(t.matrix)
                self.assertListAlmostEqual(q, t.q)


    def test_rotation_matrix_random(self):
        from random import Random
        r = Random(0).random
        for i in range(1000):
            with self.subTest(test=i):
                q = quaternion.normalize([r(), r(), r(), r()])
                rotmat = quaternion.rotation_matrix(q)
                qq = quaternion.from_rotation_matrix(rotmat)
                self.assertListAlmostEqual(q, qq)

    def test_angle_between(self):
        # ver96aug2 - 14bdc22e-92d9-4ef5-b459-7469724dea28 - ValueError: math domain error
        # AssertionError: 1.0000000006479999
        a = [1.7878716536074317e-05, 1.7878705030082886e-05, -1.656903543248786e-12, 0.9999999997613517]
        b = [1.7878712250642573e-05, 1.7878701928288437e-05, 3.354137256707306e-13, 0.9999999997613518]
        quaternion.angle_between(a, b)

    def test_slerp__border_cases(self):
        A = [random.random() for i in range(4)]
        B = [random.random() for i in range(4)]
        norm_A = math.sqrt(sum(value**2 for value in A))
        norm_B = math.sqrt(sum(value**2 for value in B))
        for i in range(4):
            A[i] /= norm_A
            B[i] /= norm_B
        C = quaternion.slerp(A, B, 0.0)
        D = quaternion.slerp(A, B, 1.0)
        for i in range(4):
            self.assertAlmostEqual(A[i], C[i], 3)
            self.assertAlmostEqual(B[i], D[i], 3)

    def test_slerp__various_angles(self):
        for a in range(40):
            angle_A = 10*math.pi*(a - 20)/40
            for b in range(10):
                angle_B = angle_A + math.pi*(b - 5)/10
                A = [0.0, 0.0, math.sin(angle_A/2), math.cos(angle_A/2)]
                B = [0.0, 0.0, math.sin(angle_B/2), math.cos(angle_B/2)]
                for c in range(10):
                    t = c / 9
                    result = quaternion.slerp(A, B, t)
                    expected_angle = (1 - t)*angle_A + t*angle_B
                    expected = [0.0, 0.0, math.sin(expected_angle/2), math.cos(expected_angle/2)]
                    for i in range(4):
                        self.assertAlmostEqual(result[i], expected[i], 3)

    def test_slerp__totally_random(self):
        for i in range(20):
            t = random.random()
            angle_A = 10*math.pi*(random.random() - 0.5)
            angle_B = angle_A + math.pi*(random.random() - 0.5)
            expected_angle = (1 - t)*angle_A + t*angle_B
            A = [0.0, 0.0, math.sin(angle_A/2), math.cos(angle_A/2)]
            B = [0.0, 0.0, math.sin(angle_B/2), math.cos(angle_B/2)]
            expected = [0.0, 0.0, math.sin(expected_angle/2), math.cos(expected_angle/2)]
            result = quaternion.slerp(A, B, t)
            for i in range(4):
                self.assertAlmostEqual(result[i], expected[i], 3)

# vim: expandtab sw=4 ts=4
