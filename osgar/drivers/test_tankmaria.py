import unittest
import datetime
from unittest.mock import MagicMock

from osgar.drivers.tankmaria import RobotTankMaria


class RobotTankMariaTest(unittest.TestCase):

    def test_parse_packet(self):
        r = RobotTankMaria({}, MagicMock())
        self.assertEqual((None, b''), r.get_packet(b''))
        self.assertEqual((b'2897 3219', b''), r.get_packet(b'2897 3219\r\n'))

    def test_invalid_packet(self):
        r = RobotTankMaria({}, MagicMock())
        self.assertEqual((None, b'\xfe'), r.get_packet(b'\xfe'))
        self.assertEqual((b'\xfe51 0 0', b''), r.get_packet(b'\xfe51 0 0\r\n'))

# vim: expandtab sw=4 ts=4
