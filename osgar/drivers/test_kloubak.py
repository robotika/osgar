import unittest
import math
from unittest.mock import MagicMock

from osgar.drivers.kloubak import compute_desired_erpm, WHEEL_DISTANCE


class KloubakTest(unittest.TestCase):

    def test_compute_desired_erpm(self):
        # go straight 1m/s
        left, right = compute_desired_erpm(1.0, 0.0)
        self.assertEqual(left, right)
        self.assertEqual(left, 917)

        wd = WHEEL_DISTANCE

        # left wheel should stay and only right one turn, say 180deg/sec
        left, right = compute_desired_erpm(math.pi * wd / 2, math.pi)
        self.assertEqual(left, 0)
        self.assertEqual(right, 1368)

        # rotate left with left moving the half the speed of right
        # left should be "wd", right "2*wd"
        left, right = compute_desired_erpm((wd/2 + wd)/2, 0.5)
        self.assertEqual(left, round(right / 2))

        # backup 0.5m/s
        left, right = compute_desired_erpm(-0.5, 0.0)
        self.assertEqual(left, right)
        self.assertEqual(left, -458)

# vim: expandtab sw=4 ts=4
