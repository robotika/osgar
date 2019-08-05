import unittest
import math
from unittest.mock import MagicMock

from osgar.drivers.kloubak import (compute_desired_erpm, compute_desired_angle,
        WHEEL_DISTANCE, compute_rear, CENTER_AXLE_DISTANCE)


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

    def test_compute_rear(self):
        speed, angular = compute_rear(1.0, 0.0, 0.0)
        self.assertAlmostEqual(speed, 1.0)
        self.assertAlmostEqual(angular, 0.0)

        # turn in place
        speed, angular = compute_rear(0.0, 0.1, math.pi/2)
        self.assertAlmostEqual(speed, 0.1 * CENTER_AXLE_DISTANCE)
        self.assertAlmostEqual(angular, 0.0)

    def test_rear_drive(self):
        # go straight
        self.assertAlmostEqual(0.0, compute_desired_angle(1.0, 0.0))

        # easy case when the joint has 60 degrees and radius is CENTER_AXLE_DISTANCE
        self.assertAlmostEqual(math.radians(60),
                compute_desired_angle(CENTER_AXLE_DISTANCE, 1.0))

        # it should be symmetrical
        self.assertAlmostEqual(math.radians(-60),
                compute_desired_angle(CENTER_AXLE_DISTANCE, -1.0))

        # perpendicular to the left, radius defined by length of joint to wheel center
#        self.assertAlmostEqual(math.radians(90),
#                compute_desired_angle(CENTER_AXLE_DISTANCE * math.radians(90), math.radians(90)))

        # the same on the other side
#        self.assertAlmostEqual(math.radians(-90),
#                compute_desired_angle(CENTER_AXLE_DISTANCE * math.radians(90), math.radians(-90)))

        # failing case "ValueError: math domain error"
        compute_desired_angle(0.1, -0.007853981633974483)

        # failing case "ValueError: math domain error" after fix - limits needed
        compute_desired_angle(0.1, 0.3063052837250049)

        # another case "ValueError: math domain error"
        angle = compute_desired_angle(0.1, -1.0637781790905438)
        self.assertAlmostEqual(angle, -math.pi)

# vim: expandtab sw=4 ts=4
