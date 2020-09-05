import unittest
import math

from moon.angle_filter import AngleFilter, AngleFilterIMU


class AngleFilterTest(unittest.TestCase):

    def test_usage(self):
        af = AngleFilter(history_size=2)
        self.assertIsNone(af.get())

        af.add(3)
        self.assertAlmostEqual(af.get(), 3)

        af.add(1)
        self.assertAlmostEqual(af.get(), 2)

        af.add(1)
        self.assertAlmostEqual(af.get(), 1)

    def test_singularity(self):
        af = AngleFilter(history_size=2)
        af.add(3)  # i.e. almost PI
        af.add(-3)
        self.assertAlmostEqual(af.get(), math.pi)

    def test_imu_filter(self):
        af = AngleFilterIMU()
        af.add(0, 0, 1)

        self.assertAlmostEqual(af.yaw.get(), 0)
        self.assertAlmostEqual(af.roll.get(), 1)

# vim: expandtab sw=4 ts=4

