import math
import copy

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
        # these values of A and B ensure their positive dot product
        A = [math.sqrt(1/10), math.sqrt(2/10), math.sqrt(3/10), math.sqrt(4/10)]
        B = [math.sqrt(1/16), math.sqrt(3/16), math.sqrt(5/16), math.sqrt(7/16)]
        remembered_A = copy.copy(A)
        remembered_B = copy.copy(B)
        C = quaternion.slerp(A, B, 0.0)
        self.assertIsInstance(C, list)
        self.assertEqual(len(C), 4)
        # expected A == C
        for i in range(4):
            self.assertAlmostEqual(A[i], C[i])
            self.assertAlmostEqual(A[i], remembered_A[i])
            self.assertAlmostEqual(B[i], remembered_B[i])
        D = quaternion.slerp(A, B, 1.0)
        self.assertIsInstance(D, list)
        self.assertEqual(len(D), 4)
        # expected B == D
        for i in range(4):
            self.assertAlmostEqual(B[i], D[i])
        # these values of A and B ensure their negative dot product
        A = [math.sqrt(1/10), math.sqrt(2/10), math.sqrt(3/10), math.sqrt(4/10)]
        B = [-math.sqrt(1/16), -math.sqrt(3/16), -math.sqrt(5/16), -math.sqrt(7/16)]
        remembered_A = copy.copy(A)
        remembered_B = copy.copy(B)
        C = quaternion.slerp(A, B, 0.0)
        self.assertIsInstance(C, list)
        self.assertEqual(len(C), 4)
        # expected A == C
        for i in range(4):
            self.assertAlmostEqual(A[i], remembered_A[i])
            self.assertAlmostEqual(B[i], remembered_B[i])
            self.assertAlmostEqual(A[i], C[i])
        D = quaternion.slerp(A, B, 1.0)
        self.assertIsInstance(D, list)
        self.assertEqual(len(D), 4)
        # expected B == -D
        for i in range(4):
            self.assertAlmostEqual(B[i], -D[i])

    def test_slerp__less_than_180(self):
        # interpolate between quaternions that differ by an angle < 180 deg
        num_steps_of_angles = 5
        num_steps_of_t = 5
        sqrt_1_7 = math.sqrt(1/7)
        sqrt_2_7 = math.sqrt(2/7)
        sqrt_4_7 = math.sqrt(4/7)
        for a in range(num_steps_of_angles):
            angle_A = a / (num_steps_of_angles - 1) * 2*math.pi
            sin_angle_A_half = math.sin(angle_A / 2)
            cos_angle_A_half = math.cos(angle_A / 2)
            for b in range(num_steps_of_angles):
                angle_B = b / (num_steps_of_angles - 1) * math.pi + angle_A
                sin_angle_B_half = math.sin(angle_B / 2)
                cos_angle_B_half = math.cos(angle_B / 2)
                for step in range(num_steps_of_t):
                    t = step / (num_steps_of_t - 1)
                    expected_angle = (1 - t)*angle_A + t*angle_B
                    sin_expected = math.sin(expected_angle/2)
                    cos_expected = math.cos(expected_angle/2)
                    # around x
                    A = [sin_angle_A_half, 0.0, 0.0, cos_angle_A_half]
                    B = [sin_angle_B_half, 0.0, 0.0, cos_angle_B_half]
                    E = [sin_expected, 0.0, 0.0, cos_expected]
                    C = quaternion.slerp(A, B, t)
                    for i in range(4):
                        self.assertAlmostEqual(C[i], E[i])
                    # around y
                    A = [0.0, sin_angle_A_half, 0.0, cos_angle_A_half]
                    B = [0.0, sin_angle_B_half, 0.0, cos_angle_B_half]
                    E = [0.0, sin_expected, 0.0, cos_expected]
                    C = quaternion.slerp(A, B, t)
                    for i in range(4):
                        self.assertAlmostEqual(C[i], E[i])
                    # around z
                    A = [0.0, 0.0, sin_angle_A_half, cos_angle_A_half]
                    B = [0.0, 0.0, sin_angle_B_half, cos_angle_B_half]
                    E = [0.0, 0.0, sin_expected, cos_expected]
                    C = quaternion.slerp(A, B, t)
                    for i in range(4):
                        self.assertAlmostEqual(C[i], E[i])
                    # around vector (sqrt(1/7), sqrt(2/7), sqrt(4/7))
                    A = [sqrt_1_7*sin_angle_A_half, sqrt_2_7*sin_angle_A_half, sqrt_4_7*sin_angle_A_half, cos_angle_A_half]
                    B = [sqrt_1_7*sin_angle_B_half, sqrt_2_7*sin_angle_B_half, sqrt_4_7*sin_angle_B_half, cos_angle_B_half]
                    E = [sqrt_1_7*sin_expected, sqrt_2_7*sin_expected, sqrt_4_7*sin_expected, cos_expected]
                    C = quaternion.slerp(A, B, t)
                    for i in range(4):
                        self.assertAlmostEqual(C[i], E[i])
                    # around vector (-sqrt(1/7), -sqrt(2/7), -sqrt(4/7))
                    A = [-sqrt_1_7*sin_angle_A_half, -sqrt_2_7*sin_angle_A_half, -sqrt_4_7*sin_angle_A_half, cos_angle_A_half]
                    B = [-sqrt_1_7*sin_angle_B_half, -sqrt_2_7*sin_angle_B_half, -sqrt_4_7*sin_angle_B_half, cos_angle_B_half]
                    E = [-sqrt_1_7*sin_expected, -sqrt_2_7*sin_expected, -sqrt_4_7*sin_expected, cos_expected]
                    C = quaternion.slerp(A, B, t)
                    for i in range(4):
                        self.assertAlmostEqual(C[i], E[i])

    def test_slerp__more_than_180(self):
        # interpolate between quaternions that differ by an angle > 180 deg
        num_steps_of_angles = 5
        num_steps_of_t = 5
        sqrt_1_7 = math.sqrt(1/7)
        sqrt_2_7 = math.sqrt(2/7)
        sqrt_4_7 = math.sqrt(4/7)
        for a in range(num_steps_of_angles):
            angle_A = a / (num_steps_of_angles - 1) * 2*math.pi
            sin_angle_A_half = math.sin(angle_A / 2)
            cos_angle_A_half = math.cos(angle_A / 2)
            # range from 1 to avoid angle difference of 180 deg
            for b in range(1, num_steps_of_angles):
                # add 180 deg to the angle difference
                angle_B = b / (num_steps_of_angles - 1) * math.pi + angle_A + math.pi
                sin_angle_B_half = math.sin(angle_B / 2)
                cos_angle_B_half = math.cos(angle_B / 2)
                for step in range(num_steps_of_t):
                    t = step / (num_steps_of_t - 1)
                    # shorten `angle_B` by 360 deg to obtain `expected_angle`
                    # in the "shorter" angle between `angle_A` and `angle_B`
                    expected_angle = (1 - t)*angle_A + t*(angle_B - 2*math.pi)
                    sin_expected = math.sin(expected_angle/2)
                    cos_expected = math.cos(expected_angle/2)
                    # around x
                    A = [sin_angle_A_half, 0.0, 0.0, cos_angle_A_half]
                    B = [sin_angle_B_half, 0.0, 0.0, cos_angle_B_half]
                    E = [sin_expected, 0.0, 0.0, cos_expected]
                    C = quaternion.slerp(A, B, t)
                    for i in range(4):
                        self.assertAlmostEqual(C[i], E[i])
                    # around y
                    A = [0.0, sin_angle_A_half, 0.0, cos_angle_A_half]
                    B = [0.0, sin_angle_B_half, 0.0, cos_angle_B_half]
                    E = [0.0, sin_expected, 0.0, cos_expected]
                    C = quaternion.slerp(A, B, t)
                    for i in range(4):
                        self.assertAlmostEqual(C[i], E[i])
                    # around z
                    A = [0.0, 0.0, sin_angle_A_half, cos_angle_A_half]
                    B = [0.0, 0.0, sin_angle_B_half, cos_angle_B_half]
                    E = [0.0, 0.0, sin_expected, cos_expected]
                    C = quaternion.slerp(A, B, t)
                    for i in range(4):
                        self.assertAlmostEqual(C[i], E[i])
                    # around vector (sqrt(1/7), sqrt(2/7), sqrt(4/7))
                    A = [sqrt_1_7*sin_angle_A_half, sqrt_2_7*sin_angle_A_half, sqrt_4_7*sin_angle_A_half, cos_angle_A_half]
                    B = [sqrt_1_7*sin_angle_B_half, sqrt_2_7*sin_angle_B_half, sqrt_4_7*sin_angle_B_half, cos_angle_B_half]
                    E = [sqrt_1_7*sin_expected, sqrt_2_7*sin_expected, sqrt_4_7*sin_expected, cos_expected]
                    C = quaternion.slerp(A, B, t)
                    for i in range(4):
                        self.assertAlmostEqual(C[i], E[i])
                    # around vector (-sqrt(1/7), -sqrt(2/7), -sqrt(4/7))
                    A = [-sqrt_1_7*sin_angle_A_half, -sqrt_2_7*sin_angle_A_half, -sqrt_4_7*sin_angle_A_half, cos_angle_A_half]
                    B = [-sqrt_1_7*sin_angle_B_half, -sqrt_2_7*sin_angle_B_half, -sqrt_4_7*sin_angle_B_half, cos_angle_B_half]
                    E = [-sqrt_1_7*sin_expected, -sqrt_2_7*sin_expected, -sqrt_4_7*sin_expected, cos_expected]
                    C = quaternion.slerp(A, B, t)
                    for i in range(4):
                        self.assertAlmostEqual(C[i], E[i])

# vim: expandtab sw=4 ts=4
