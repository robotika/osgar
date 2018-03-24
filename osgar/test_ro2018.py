import unittest
from unittest.mock import MagicMock
import math
from datetime import timedelta

from osgar.ro2018 import (geo_length, geo_angle, LogBusHandler, LogBusHandlerInputsOnly,
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

    def test_log_bus_handler(self):
        log = MagicMock()
        log_data = [
            (timedelta(microseconds=10), 1, b'(1,2)'),
            (timedelta(microseconds=11), 1, b'(3,4,5)'),
            (timedelta(microseconds=30), 2, b'[8,9]'),
        ]
        log.read_gen = MagicMock(return_value=iter(log_data))
        inputs = {1:'raw'}
        outputs = {2:'can'}
        bus = LogBusHandler(log, inputs, outputs)
        bus.listen()
        with self.assertRaises(AssertionError) as e:
            bus.publish('can', b'parsed data')
        self.assertEqual(str(e.exception), "(b'parsed data', [8, 9])")

    def test_wrong_publish_channel(self):
        log = MagicMock()
        log_data = [
            (timedelta(microseconds=10), 1, b'(1,2)'),
            (timedelta(microseconds=30), 2, b'[8,9]'),
            (timedelta(microseconds=35), 3, b'[8,9]'),
        ]
        log.read_gen = MagicMock(return_value=iter(log_data))
        inputs = {1:'raw'}
        outputs = {2:'can', 3:'can2'}
        bus = LogBusHandler(log, inputs, outputs)
        bus.listen()
        with self.assertRaises(AssertionError) as e:
            bus.publish('can2', [8, 9])
        self.assertEqual(str(e.exception), "('can2', 'can')")

        with self.assertRaises(AssertionError) as e:
            bus.publish('can3', [1, 2])
        self.assertEqual(str(e.exception), "('can3', dict_values(['can', 'can2']))")

    def test_log_bus_handler_inputs_onlye(self):
        log = MagicMock()
        log_data = [
            (timedelta(microseconds=10), 1, b'(1,2)'),
            (timedelta(microseconds=11), 1, b'(3,4,5)'),
            (timedelta(microseconds=30), 2, b'[8,9]'),
        ]
        log.read_gen = MagicMock(return_value=iter(log_data))
        inputs = {1:'raw'}
        bus = LogBusHandlerInputsOnly(log, inputs)
        self.assertEqual(bus.listen(), (timedelta(microseconds=10), 'raw', (1, 2)))
        bus.publish('new_channel', b'some data')
        self.assertEqual(bus.listen(), (timedelta(microseconds=11), 'raw', (3, 4, 5)))

    def test_log_bus_handler_raw_channels(self):
        log = MagicMock()
        log_data = [
            (timedelta(microseconds=10), 1, b'(1,2)'),
            (timedelta(microseconds=11), 1, b'(3,4,5)'),
            (timedelta(microseconds=30), 2, b'[8,9]'),
        ]
        log.read_gen = MagicMock(return_value=iter(log_data))
        inputs = {1:'raw'}
        bus = LogBusHandlerInputsOnly(log, inputs, raw_channels=['raw'])
        self.assertEqual(bus.listen(), (timedelta(microseconds=10), 'raw', b'(1,2)'))
        bus.publish('new_channel', b'some data')
        self.assertEqual(bus.listen(), (timedelta(microseconds=11), 'raw', b'(3,4,5)'))

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
