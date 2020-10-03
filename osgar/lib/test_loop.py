import math

from osgar.lib.loop import LoopDetector
from osgar.lib.quaternion import euler_to_quaternion
from osgar.lib.unittest import TestCase

class LoopDetectionTest(TestCase):
    def test_perfect_loop(self):
        forward = euler_to_quaternion(0, 0, 0)
        trajectory = [((0, 0, 0), forward),
                      ((2, 0, 0), forward),
                      ((2, 2, 0), forward),
                      ((0, 2, 0), forward),
                      ((0, 0, 0), forward)]
        detector = LoopDetector(min_loop_length=5)
        detector.add_all(trajectory)
        loop = detector.loop()
        self.assertEqual(loop, trajectory)


    def test_loop_with_prefix(self):
        forward = euler_to_quaternion(0, 0, 0)
        trajectory = [((-1, 0, 0), forward),
                      ((0, 0, 0), forward),
                      ((2, 0, 0), forward),
                      ((2, 2, 0), forward),
                      ((0, 2, 0), forward),
                      ((0, 0, 0), forward)]
        detector = LoopDetector(min_loop_length=5)
        detector.add_all(trajectory)
        loop = detector.loop()
        self.assertEqual(loop, trajectory[1:])


    def test_loop_with_small_rotation(self):
        forward = euler_to_quaternion(0, 0, 0)
        almost_forward = euler_to_quaternion(math.radians(10), 0, 0)
        trajectory = [((0, 0, 0), forward),
                      ((2, 0, 0), forward),
                      ((2, 2, 0), forward),
                      ((0, 2, 0), forward),
                      ((0, 0, 0), almost_forward)]
        detector = LoopDetector(min_loop_length=5)
        detector.add_all(trajectory)
        loop = detector.loop()
        self.assertEqual(loop, trajectory)


    def test_loop_with_small_drift(self):
        forward = euler_to_quaternion(0, 0, 0)
        trajectory = [((0, 0, 0), forward),
                      ((2, 0, 0), forward),
                      ((2, 2, 0), forward),
                      ((0, 2, 0), forward),
                      ((0.1, 0.1, 0.1), forward)]
        detector = LoopDetector(min_loop_length=5)
        detector.add_all(trajectory)
        loop = detector.loop()
        self.assertEqual(loop, trajectory)


    def test_granularity(self):
        forward = euler_to_quaternion(0, 0, 0)
        trajectory = [((0, 0, 0), forward),
                      ((1, 0, 0), forward),
                      ((2, 0, 0), forward),
                      ((2, 1, 0), forward),
                      ((2, 2, 0), forward),
                      ((1, 2, 0), forward),
                      ((0, 2, 0), forward),
                      ((0, 1, 0), forward),
                      ((0, 0, 0), forward)]
        detector = LoopDetector(min_loop_length=5, granularity=2.0)
        detector.add_all(trajectory)
        loop = detector.loop()
        self.assertEqual(loop, trajectory[::2])


    def test_not_a_loop_too_far_horizontally(self):
        forward = euler_to_quaternion(0, 0, 0)
        trajectory = [((0, 0, 0), forward),
                      ((2, 0, 0), forward),
                      ((2, 2, 0), forward),
                      ((0, 2, 0), forward)]
        detector = LoopDetector(min_loop_length=5)
        detector.add_all(trajectory)
        loop = detector.loop()
        self.assertIsNone(loop)


    def test_not_a_loop_too_far_vertically(self):
        forward = euler_to_quaternion(0, 0, 0)
        trajectory = [((0, 0, 0), forward),
                      ((2, 0, 0), forward),
                      ((2, 2, 0), forward),
                      ((0, 2, 0), forward),
                      ((0, 0, 1), forward)]
        detector = LoopDetector(min_loop_length=5)
        detector.add_all(trajectory)
        loop = detector.loop()
        self.assertIsNone(loop)


    def test_not_a_loop_different_direction(self):
        forward = euler_to_quaternion(0, 0, 0)
        left = euler_to_quaternion(math.pi/2, 0, 0)
        trajectory = [((0, 0, 0), forward),
                      ((2, 0, 0), forward),
                      ((2, 2, 0), forward),
                      ((0, 2, 0), forward),
                      ((0, 0, 0), left)]
        detector = LoopDetector(min_loop_length=5)
        detector.add_all(trajectory)
        loop = detector.loop()
        self.assertIsNone(loop)


    def test_not_a_loop_too_short(self):
        forward = euler_to_quaternion(0, 0, 0)
        trajectory = [((0, 0, 0), forward),
                      ((1, 0, 0), forward),
                      ((1, 1, 0), forward),
                      ((0, 1, 0), forward),
                      ((0, 0, 0), forward)]
        detector = LoopDetector(min_loop_length=5)
        detector.add_all(trajectory)
        loop = detector.loop()
        self.assertIsNone(loop)


    def test_not_a_loop_too_long(self):
        forward = euler_to_quaternion(0, 0, 0)
        trajectory = [((0, 0, 0), forward),
                      ((100, 0, 0), forward),
                      ((100, 100, 0), forward),
                      ((0, 100, 0), forward),
                      ((0, 0, 0), forward)]
        detector = LoopDetector(min_loop_length=5)
        detector.add_all(trajectory)
        loop = detector.loop()
        self.assertIsNone(loop)


if __name__ == '__main__':
    import unittest
    unittest.main()
