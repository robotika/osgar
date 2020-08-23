import unittest

from moon.odometry import Odometry


class MoonOdometryTest(unittest.TestCase):

    def test_usage(self):
        odom = Odometry()
        self.assertEqual(odom.pose2d, (0, 0, 0))


# vim: expandtab sw=4 ts=4

