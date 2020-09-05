import unittest
import math
from unittest.mock import MagicMock

from osgar.drivers.kloubak import (compute_desired_erpm, compute_desired_angle,
        WHEEL_DISTANCE, compute_rear, CENTER_AXLE_DISTANCE, RobotKloubak,
        get_downdrop_bumpers, MAX_JOINT_ANGLE, CAN_ID_ENCODERS)


class KloubakTest(unittest.TestCase):

    def test_compute_desired_erpm(self):
        # go straight 1m/s
        left, right = compute_desired_erpm(1.0, 0.0)
        self.assertEqual(left, right)
        self.assertEqual(left, 694)  # 917 for 25" wheels and 694 for 33"

        wd = WHEEL_DISTANCE

        # left wheel should stay and only right one turn, say 180deg/sec
        left, right = compute_desired_erpm(math.pi * wd / 2, math.pi)
        self.assertEqual(left, 0)
        self.assertEqual(right, 1082)  # 1428 for 25" wheels and 1082 for 33"

        # rotate left with left moving the half the speed of right
        # left should be "wd", right "2*wd"
        left, right = compute_desired_erpm((wd/2 + wd)/2, 0.5)
        self.assertAlmostEqual(left, right / 2, 0)

        # backup 0.5m/s
        left, right = compute_desired_erpm(-0.5, 0.0)
        self.assertEqual(left, right)
        self.assertEqual(left, -347)  # -458 for 25" wheels and -347 for 33"

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
        self.assertAlmostEqual(0.9272952,
                compute_desired_angle(CENTER_AXLE_DISTANCE, 0.5))

        # it should be symmetrical
        self.assertAlmostEqual(-0.9272952,
                compute_desired_angle(CENTER_AXLE_DISTANCE, -0.5))

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
        self.assertAlmostEqual(angle, -1.1868238913561442)

    def test_invalid_can_message(self):
        # this message killed Kloubak K2 on DARPA SubT Tunnel Circuit, Day 2
        config = {}
        bus = MagicMock()
        k2 = RobotKloubak(config, bus)
        # this used to assert:
        #      assert len(data) == 1, len(data)
        #   AssertionError: 8
        k2.update_buttons(b'\x00\x00\x04\x1f\x00\x1f\x00\x7f')
        self.assertEqual(k2.can_errors, 1)

        k2.update_encoders(msg_id=0x92, data=b'\x97')

    def test_partial_axis_encoders_update(self):
        config = {
            'num_axis': 3
        }
        bus = MagicMock()
        k3 = RobotKloubak(config, bus)
        self.assertIsNone(k3.partial_axis_encoders_update((121, 2, 2280, 2486)))

        # epoch change, but incomplete data -> do not return anything
        self.assertIsNone(k3.partial_axis_encoders_update((122, 1, 2183, 3786)))

        # now proper update
        self.assertIsNone(k3.partial_axis_encoders_update((122, 2, 2280, 2486)))
        self.assertIsNone(k3.partial_axis_encoders_update((122, 3, 2164, 2486)))

        # epoch change
        self.assertEqual(k3.partial_axis_encoders_update((123, 1, 2183, 3786)),
                         [2183, 3786, 2280, 2486, 2164, 2486])

    def test_downdrops(self):
        self.assertEqual(get_downdrop_bumpers([500, 500]), [False, False])
        self.assertEqual(get_downdrop_bumpers([1000, 200]), [True, True])

    def test_compute_desired_angle_division_by_zero(self):
        self.assertAlmostEqual(compute_desired_angle(desired_speed=0.0, desired_angular_speed=0.1),
                               MAX_JOINT_ANGLE)

        self.assertAlmostEqual(compute_desired_angle(desired_speed=0.0, desired_angular_speed=-20.1),
                               -MAX_JOINT_ANGLE)

    def test_publish_encoders(self):
        config = {
            'num_axis': 3
        }
        bus = MagicMock()
        k3 = RobotKloubak(config, bus)

        flags = 0
        k3.process_packet([CAN_ID_ENCODERS, bytes([13, 1, 0, 0, 0, 0]), flags])
        bus.publish.assert_not_called()
        k3.process_packet([CAN_ID_ENCODERS, bytes([13, 2, 0, 0, 0, 0]), flags])
        bus.publish.assert_not_called()
        k3.process_packet([CAN_ID_ENCODERS, bytes([13, 3, 0, 0, 0, 0]), flags])
        bus.publish.assert_not_called()

        k3.process_packet([CAN_ID_ENCODERS, bytes([14, 1, 0, 1, 0, 2]), flags])
        bus.publish.assert_called()

# vim: expandtab sw=4 ts=4
