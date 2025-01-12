
import unittest
from logging import raiseExceptions
from unittest.mock import MagicMock
from datetime import timedelta

from osgar.platforms.matty import Matty


class MattyTest(unittest.TestCase):

    def test_usage(self):
        bus = MagicMock()
        robot = Matty(bus=bus, config={})
        robot.on_tick(None)
#        robot.on_esp_data(b'1234')
#        bus.publish.assert_called_with('emergency_stop', True)

    def test_crc(self):
        bus = MagicMock()
        robot = Matty(bus=bus, config={})
        # packet 0 is not acknowledged
        with self.assertRaises(AssertionError):
            robot.on_esp_data(bytes.fromhex('55 02 00 4e b0'))

        # ACK and proper counter
        robot.counter = 1
        robot.on_esp_data(bytes.fromhex('55020141bc'))

    def test_send_esp(self):
        bus = MagicMock()
        robot = Matty(bus=bus, config={})
        robot.send_esp(b'S')
        bus.publish.assert_called_with('esp_data', bytes().fromhex('55020153aa'))
