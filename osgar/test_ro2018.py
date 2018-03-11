import unittest
import math

from ro2018 import geo_length, geo_angle


class RO2018Test(unittest.TestCase):

    def test_geo_lenght(self):
        self.assertAlmostEqual(
                geo_length((51749517, 180462688), (51749517, 180462688)),
                0.0)
        self.assertLess(
                geo_length((51749517, 180462688), (51749518, 180462688)),
                0.03)  # i.e. resolution ~ 3cm

    def test_geo_angle(self):
        self.assertAlmostEqual(
                geo_angle((51749517, 180462688), (51749617, 180462688)),
                0.0)  # go East
        self.assertAlmostEqual(
                geo_angle((51749517, 180462688), (51749517, 180462788)),
                math.radians(90.0))  # go North
        self.assertIsNone(
                geo_angle((51749517, 180462688), (51749518, 180462688)))
                # for too small distance

# vim: expandtab sw=4 ts=4
