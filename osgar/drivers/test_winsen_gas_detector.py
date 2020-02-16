import unittest
import datetime
from unittest.mock import MagicMock

from osgar.drivers.winsen_gas_detector import WinsenCO2


class WinsenCO2Test(unittest.TestCase):

    def test_parse_packet(self):
        ref_packet = bytes.fromhex('ff86063b3d000000fc')
        sensor = WinsenCO2(config={}, bus=MagicMock())
        sensor._buf += ref_packet
        packet = sensor.get_packet()
        self.assertEqual(packet, ref_packet)

        co2_value = sensor.parse_CO2_packet(packet)
        self.assertEqual(co2_value, 1595)

    def test_clear_buffer(self):
        sensor = WinsenCO2(config={}, bus=MagicMock())
        sensor._buf = bytes([0] * 1000)
        packet = sensor.get_packet()
        self.assertIsNone(packet)
        self.assertEqual(sensor._buf, b'')

    def test_keep_ff_in_buffer(self):
        sensor = WinsenCO2(config={}, bus=MagicMock())
        sensor._buf = bytes([0] * 1000) + b'\xFF'
        packet = sensor.get_packet()
        self.assertIsNone(packet)
        self.assertEqual(sensor._buf, bytes([0xFF]))

    def test_checksum_error(self):
        sensor = WinsenCO2(config={}, bus=MagicMock())
        sensor._buf = bytes.fromhex('ff86063b3d000000fe')  # valid should be 0xFC
        packet = sensor.get_packet()
        self.assertIsNone(packet)

        self.assertEqual(sensor.errors, 1)

# vim: expandtab sw=4 ts=4
