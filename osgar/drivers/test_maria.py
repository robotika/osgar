import unittest
import datetime
from unittest.mock import MagicMock

from osgar.drivers.maria import RobotMaria, SpeedControl, MAX_PWM


class RobotTankMariaTest(unittest.TestCase):

    def test_parse_packet(self):
        r = RobotMaria({}, MagicMock())
        self.assertEqual((None, b''), r.get_packet(b''))
        self.assertEqual((b'2897 3219', b''), r.get_packet(b'2897 3219\r\n'))

    def test_invalid_packet(self):
        r = RobotMaria({}, MagicMock())
        self.assertEqual((None, b'\xfe'), r.get_packet(b'\xfe'))
        self.assertEqual((b'\xfe51 0 0', b''), r.get_packet(b'\xfe51 0 0\r\n'))

    def test_speed_control(self):
        sc = SpeedControl()
        self.assertEqual(sc.update(0), 0)

        sc.set_desired_ticks(50)
        self.assertEqual(sc.update(0), 10)  # return MIN_PWM
        self.assertEqual(sc.update(0), 12)  # speed up by step
        self.assertEqual(sc.update(50), 12) # keep last pwm
        self.assertEqual(sc.update(70), 10) # slow down by step

        # change polarity
        sc.set_desired_ticks(-50)
        self.assertEqual(sc.update(50), 0)  # wait until it is not moving wrong direction
        self.assertEqual(sc.update(3), -10) # -MIN_PWM
        self.assertEqual(sc.update(-30), -12)  # -(MIN_PWM + step)
        self.assertEqual(sc.update(-51), -12)  # last_pwm (encoders is within tolerance)
        self.assertEqual(sc.update(-66), -10)  # last_pwm - step

        for __ in range(20):
            self.assertGreaterEqual(sc.update(-10), -MAX_PWM)

        sc.set_desired_ticks(50)
        self.assertEqual(sc.update(-70), 0)
        self.assertEqual(sc.update(-2), 10)
        self.assertEqual(sc.update(-2), 12)

# vim: expandtab sw=4 ts=4
