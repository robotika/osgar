
import unittest
from unittest.mock import MagicMock
from datetime import timedelta

from osgar.platforms.yuhesen import FR07


class FR07Test(unittest.TestCase):

    def test_usage(self):
        bus = MagicMock()
        robot = FR07(bus=bus, config={})
        robot.on_can([0x18c4eaef, bytes.fromhex('3200000000704002'), 1])
        bus.publish.assert_called_with('emergency_stop', True)

    def test_control_speed(self):
        bus = MagicMock()
        robot = FR07(bus=bus, config={})
        robot.desired_speed = 5.0  # m/s (from user manual)
        robot.on_can([0x18c4d2ef, bytes.fromhex('0100700000102041'), 1])
        bus.publish.assert_called_with('can', [0x18C4D2D0, bytes.fromhex('843801000000209d'), 1])

    def test_control_steering(self):
        bus = MagicMock()
        robot = FR07(bus=bus, config={})
        robot.desired_steering_angle_deg = -25.0  # deg (from user manual)
        robot.on_can([0x18c4d2ef, bytes.fromhex('0100700000102041'), 1])
        bus.publish.assert_called_with('can', [0x18C4D2D0, bytes.fromhex('0400c0630f002088'), 1])
