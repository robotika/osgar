import unittest
from unittest.mock import MagicMock
import math

from osgar.ro2018 import (geo_length, geo_angle,
                          latlon2xy, EmergencyStopMonitor, EmergencyStopException)


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

    def test_latlon2xy(self):
        self.assertEqual(latlon2xy(50.128246666666, 14.3748658333333), (51749517, 180462688 - 1000))

    def test_emergency_stop_monitor(self):
        robot = MagicMock()
        robot.register = MagicMock(return_value=42)
        with EmergencyStopMonitor(robot) as esm:
            robot.register.assert_called_once_with(esm.update)
            robot.status = 0x0
            with self.assertRaises(EmergencyStopException):
                esm.update(robot)
        robot.unregister.assert_called_once_with(42)

# vim: expandtab sw=4 ts=4
