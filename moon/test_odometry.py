import unittest
import math

from moon.odometry import Odometry


class MoonOdometryTest(unittest.TestCase):

    def test_usage(self):
        odom = Odometry()
        self.assertEqual(odom.pose2d, (0, 0, 0))

        names = [b'bl_arm_joint', b'bl_steering_arm_joint', b'bl_wheel_joint',
                b'br_arm_joint', b'br_steering_arm_joint', b'br_wheel_joint',
                b'fl_arm_joint', b'fl_steering_arm_joint', b'fl_wheel_joint',
                b'fr_arm_joint', b'fr_steering_arm_joint', b'fr_wheel_joint',
                b'sensor_joint']

        data = [0.045406858648845194, 0.004015732023318286, 15.865968131515782,
                0.06465937149762269, -0.004300069050040101, 15.749286284936085,
                -0.06585596104286928, -0.002379553028373671,
                16.106778719739687, -0.057778736914743334,
                0.001733177352261528, 15.999004953177929,
                -0.00017593211588806668]

        odom.update_joint_position(names, data)
        self.assertEqual(odom.pose2d, (0, 0, 0))

        # current behavior - ignore steering angles
        data[names.index(b'bl_wheel_joint')] += 1.0
        data[names.index(b'br_wheel_joint')] += 1.0
        odom.update_joint_position(names, data)
        self.assertAlmostEqual(odom.pose2d[0], 0.275/2)
        self.assertAlmostEqual(odom.pose2d[1], 0, 4)
        self.assertAlmostEqual(odom.pose2d[2], 0)

    def test_is_crab_step(self):
        odom = Odometry()
        self.assertTrue(odom.is_crab_step([0, 0, 0, 0]))
        a = math.radians(45)
        self.assertFalse(odom.is_crab_step([a, a, -a, -a]))

    def test_steering(self):
        odom = Odometry()
        self.assertEqual(odom.pose2d, (0, 0, 0))

        names = [b'bl_arm_joint', b'bl_steering_arm_joint', b'bl_wheel_joint',
                b'br_arm_joint', b'br_steering_arm_joint', b'br_wheel_joint',
                b'fl_arm_joint', b'fl_steering_arm_joint', b'fl_wheel_joint',
                b'fr_arm_joint', b'fr_steering_arm_joint', b'fr_wheel_joint',
                b'sensor_joint']

        data = [0.0, 0.0, 10.0,
                0.0, 0.0, 10.0,
                0.0, 0.0, 10.0,
                0.0, 0.0, 10.0,
                0.0]

        odom.update_joint_position(names, data)
        self.assertEqual(odom.pose2d, (0, 0, 0))

        data = [0.0, 0.0, 11.0,
                0.0, 0.0, 11.0,
                0.0, 0.0, 11.0,
                0.0, 0.0, 11.0,
                0.0]

        odom.update_joint_position(names, data)
        self.assertEqual(odom.pose2d, (0.275, 0, 0))

        a90 = math.radians(90)
        data = [0.0, a90, 12.0,
                0.0, a90, 12.0,
                0.0, a90, 12.0,
                0.0, a90, 12.0,
                0.0]
        odom.update_joint_position(names, data)
        self.assertEqual(odom.pose2d, (0.275, 0.275, 0))

    def test_is_turn_in_place(self):
        odom = Odometry()
        self.assertFalse(odom.is_turn_in_place([0, 0, 0, 0]))
        a = math.radians(45)
        self.assertTrue(odom.is_turn_in_place([-a, a, a, -a]))

    def test_is_on_circle(self):
        odom = Odometry()
        a = math.radians(45)
        self.assertFalse(odom.is_on_circle([a, a, a, a]))  # crab
        b = math.atan2(1, 3)  # for rover approx 2x2m
        self.assertTrue(odom.is_on_circle([a, b, -a, -b]))

    def test_circle_update(self):
        odom = Odometry()
        steering = [math.radians(a) for a in [5, 4, -5, -4]]
        drive = [x/1000.0 for x in [40, 44, 36, 44]]
        angle, dist = odom.circle_update(steering, drive)
        self.assertAlmostEqual(angle, 0.004052072)  # i.e. it is turning
        self.assertAlmostEqual(dist, 0.041)

# vim: expandtab sw=4 ts=4

