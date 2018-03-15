import unittest
from unittest.mock import MagicMock
import math
from datetime import timedelta

from osgar.ro2018 import geo_length, geo_angle, LogBusHandler


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

# vim: expandtab sw=4 ts=4
