import unittest
import math

from moon.angle_filter import AngleFilter


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

# vim: expandtab sw=4 ts=4

