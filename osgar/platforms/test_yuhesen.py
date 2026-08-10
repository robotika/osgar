
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
        bus.publish.assert_any_call('can', [0x18C4D2D0, bytes.fromhex('843801000000209d'), 1])

        robot.on_can([0x18c4d2ef, bytes.fromhex('843801000000209d'), 1])
        self.assertEqual(robot.last_speed, 5000)

    def test_control_steering(self):
        bus = MagicMock()
        robot = FR07(bus=bus, config={})
        robot.desired_steering_angle_deg = -25.0  # deg (from user manual)
        robot.on_can([0x18c4d2ef, bytes.fromhex('0100700000102041'), 1])
        bus.publish.assert_any_call('can', [0x18C4D2D0, bytes.fromhex('0400c0630f002088'), 1])

        robot.on_can([0x18c4d2ef, bytes.fromhex('0400c0630f002088'), 1])
        self.assertEqual(robot.last_steering, -2500)

    def test_manual_mode(self):
        bus = MagicMock()
        robot = FR07(bus=bus, config={})

        # Vehicle mode 1 (remote/manual) -> publish True
        robot.on_can([0x18c4d2ef, bytes.fromhex('0100700000102041'), 1])
        bus.publish.assert_any_call('manual', True)
        bus.publish.reset_mock()

        # Vehicle mode 1 again -> no change, do not publish
        robot.on_can([0x18c4d2ef, bytes.fromhex('0100700000102041'), 1])
        for call in bus.publish.call_args_list:
            self.assertNotEqual(call[0][0], 'manual')
        bus.publish.reset_mock()

        # Vehicle mode 0 (auto) -> manual is False -> publish False
        robot.on_can([0x18c4d2ef, bytes.fromhex('0100700000002051'), 1])
        bus.publish.assert_any_call('manual', False)
        bus.publish.reset_mock()

        # Vehicle mode 2 (stop) -> manual is False -> no change, do not publish
        robot.on_can([0x18c4d2ef, bytes.fromhex('0100700000202071'), 1])
        for call in bus.publish.call_args_list:
            self.assertNotEqual(call[0][0], 'manual')

    def test_bumpers(self):
        bus = MagicMock()
        robot = FR07(bus=bus, config={})

        # Initially, both bumpers are None. On first msg with 0x00, they both change to False
        # payload: 0000010000001011 (last byte 0x11 is checksum)
        robot.on_can([0x18c4daef, bytes.fromhex('0000010000001011'), 1])
        bus.publish.assert_any_call('bumpers_front', False)
        bus.publish.assert_any_call('bumpers_rear', False)
        bus.publish.reset_mock()

        # Front bumper active: 0x02.
        # payload: 0000010200001013 (last byte 0x13 is checksum)
        robot.on_can([0x18c4daef, bytes.fromhex('0000010200001013'), 1])
        bus.publish.assert_any_call('bumpers_front', True)
        # bumpers_rear should not be published as it remains False
        for call in bus.publish.call_args_list:
            self.assertNotEqual(call[0][0], 'bumpers_rear')
        bus.publish.reset_mock()

        # Rear bumper active, front inactive: 0x10.
        # payload: 0000011000001001 (last byte 0x01 is checksum)
        robot.on_can([0x18c4daef, bytes.fromhex('0000011000001001'), 1])
        bus.publish.assert_any_call('bumpers_front', False)
        bus.publish.assert_any_call('bumpers_rear', True)
        bus.publish.reset_mock()

        # Same state again: no change, nothing should be published
        robot.on_can([0x18c4daef, bytes.fromhex('0000011000001001'), 1])
        for call in bus.publish.call_args_list:
            self.assertIn(call[0][0], ['can'])  # only command response can is published
        bus.publish.reset_mock()

    def test_brakes(self):
        bus = MagicMock()
        robot = FR07(bus=bus, config={})

        # Initially, brakes is None. On first msg with 0x00, it changes to False
        # payload: 0000010000001011 (last byte 0x11 is checksum)
        robot.on_can([0x18c4daef, bytes.fromhex('0000010000001011'), 1])
        bus.publish.assert_any_call('brakes', False)
        bus.publish.reset_mock()

        # Brakes active (bit 4 of byte 1 is set -> 0x10):
        # payload: 0010010000002031 (counter 0x20, checksum 0x31)
        robot.on_can([0x18c4daef, bytes.fromhex('0010010000002031'), 1])
        bus.publish.assert_any_call('brakes', True)
        bus.publish.reset_mock()

        # Same state again (brakes still active):
        # payload: 0010010000003021 (counter 0x30, checksum 0x21)
        robot.on_can([0x18c4daef, bytes.fromhex('0010010000003021'), 1])
        for call in bus.publish.call_args_list:
            self.assertNotEqual(call[0][0], 'brakes')
        bus.publish.reset_mock()

        # Brakes inactive:
        # payload: 0000010000004041 (counter 0x40, checksum 0x41)
        robot.on_can([0x18c4daef, bytes.fromhex('0000010000004041'), 1])
        bus.publish.assert_any_call('brakes', False)

    def test_gear(self):
        bus = MagicMock()
        robot = FR07(bus=bus, config={})

        # Initially, gear is None. On first msg with 0x01 (P), it changes to 1 (GearP)
        # payload: 0100700000102041 (counter 0x20, checksum 0x41)
        robot.on_can([0x18c4d2ef, bytes.fromhex('0100700000102041'), 1])
        bus.publish.assert_any_call('gear', 1)
        bus.publish.reset_mock()

        # Same state again (gear still 1):
        # payload: 0100700000103051 (counter 0x30, checksum 0x51)
        robot.on_can([0x18c4d2ef, bytes.fromhex('0100700000103051'), 1])
        for call in bus.publish.call_args_list:
            self.assertNotEqual(call[0][0], 'gear')
        bus.publish.reset_mock()

        # Gear active with value 4 (D):
        # payload: 0400700000104024 (counter 0x40, checksum 0x24)
        robot.on_can([0x18c4d2ef, bytes.fromhex('0400700000104024'), 1])
        bus.publish.assert_any_call('gear', 4)
        bus.publish.reset_mock()

        # Gear active with value 0 (disable):
        # payload: 0000700000105030 (counter 0x50, checksum 0x30)
        robot.on_can([0x18c4d2ef, bytes.fromhex('0000700000105030'), 1])
        bus.publish.assert_any_call('gear', 0)


