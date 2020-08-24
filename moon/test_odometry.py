import unittest

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
        self.assertAlmostEqual(odom.pose2d[0], 0.275)
        self.assertAlmostEqual(odom.pose2d[1], 0)
        self.assertAlmostEqual(odom.pose2d[2], 0)

# vim: expandtab sw=4 ts=4

