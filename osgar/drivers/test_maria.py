import unittest
import datetime
from unittest.mock import MagicMock

from osgar.drivers.tankmaria import RobotTankMaria, SpeedControl, MAX_PWM


class RobotTankMariaTest(unittest.TestCase):

    def test_parse_packet(self):
        r = RobotTankMaria({}, MagicMock())
        self.assertEqual((None, b''), r.get_packet(b''))
        self.assertEqual((b'2897 3219', b''), r.get_packet(b'2897 3219\r\n'))

    def test_invalid_packet(self):
        r = RobotTankMaria({}, MagicMock())
        self.assertEqual((None, b'\xfe'), r.get_packet(b'\xfe'))
        self.assertEqual((b'\xfe51 0 0', b''), r.get_packet(b'\xfe51 0 0\r\n'))

    def test_speed_control(self):
        sc = SpeedControl()
        self.assertEqual(sc.update(0), 0)

        sc.set_desired_ticks(50)
        self.assertEqual(sc.update(0), 100)
        self.assertEqual(sc.update(0), 110)
        self.assertEqual(sc.update(50), 110)
        self.assertEqual(sc.update(70), 100)

        # change polarity
        sc.set_desired_ticks(-50)
        self.assertEqual(sc.update(50), 0)  # wait until it is not moving wrong direction
        self.assertEqual(sc.update(3), -100)
        self.assertEqual(sc.update(-30), -110)
        self.assertEqual(sc.update(-51), -110)
        self.assertEqual(sc.update(-66), -100)

        for __ in range(20):
            self.assertGreaterEqual(sc.update(-10), -MAX_PWM)

        sc.set_desired_ticks(50)
        self.assertEqual(sc.update(-70), 0)
        self.assertEqual(sc.update(-2), 100)
        self.assertEqual(sc.update(-2), 110)

# vim: expandtab sw=4 ts=4
