import unittest
from unittest.mock import MagicMock

from osgar.drivers.rosmsg import (ROSMsgParser, parse_volatile, parse_bucket, parse_topic)


class ROSMsgParserTest(unittest.TestCase):

    def test_parse_volatile(self):
        data = b"<\x00\x00\x00\x8c\x00\x00\x009\x00\x00\x00\x00'\xb9)\x17\x00\x00\x00" + \
               b"scout_1/volatile_sensor\x08\x00\x00\x00methanol\n\x00\x00\x00\x01\xfb\xde`?"
        self.assertEqual(parse_volatile(data), ['methanol', 0.8784024119377136, 10])

    def test_parse_bucket(self):
        data = b'\x16\x00\x00\x00\n\x00\x00\x00sulfur_dio\x1b\x00\x00\x00\xd3\xa7\xabA'
        self.assertEqual(parse_bucket(data), ['sulfur_dio', 27, 21.456945419311523])

    def test_parse_bin(self):
        data = b'\x82\x00\x00\x00\x03\x00\x00\x00ice\x06\x00\x00\x00ethene\x07\x00\x00\x00methane\x0b\x00\x00\x00carbon_mono\n\x00\x00\x00carbon_dio\x07\x00\x00\x00ammonia\x0c\x00\x00\x00hydrogen_sul\n\x00\x00\x00sulfur_dio\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00'
        self.assertEqual(parse_topic('srcp2_msgs/HaulerMsg', data), [['ice', 'ethene', 'methane', 'carbon_mono', 'carbon_dio', 'ammonia', 'hydrogen_sul', 'sulfur_dio'], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])

    def test_parse_score_qual2(self):
        data = b'\x87\x00\x00\x00\x03\x00\x00\x00ice\x06\x00\x00\x00ethene\x07\x00\x00\x00methane' + \
               b'\x08\x00\x00\x00carbon_mono\n\x00\x00\x00carbon_dio\x07\x00\x00\x00ammonia\x0c\x00\x00\x00' + \
               b'hydrogen_sul\n\x00\x00\x00sulfur_dio\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00' + \
               b'\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00' + \
               b'\x00\x00\x00\x00\x00\x00\x00\x00\x00'
        self.assertEqual(parse_topic('srcp2_msgs/Qual2ScoringMsg', data), [0, 0])

    def test_radio(self):
        data = b'radio X30F60R [0, 100, 30]\n'
        bus = MagicMock()
        r = ROSMsgParser(config={}, bus=bus)
        r.slot_raw(timestamp=None, data=data)
        bus.publish.assert_called_with('radio', [b'X30F60R', b'[0, 100, 30]\n'])

    def test_publish_desired_speed(self):
        data = b'\x08\x00\x00\x00\x02\x00\x00\x00\x00\x06\x81\x14'  # clock as 8 bytes
        bus = MagicMock()
        r = ROSMsgParser(config={}, bus=bus)
        r.slot_raw(timestamp=None, data=data)
        bus.publish.assert_called_with('sim_time_sec', 2)

        clock_data = b'\x08\x00\x00\x00\x03\x00\x00\x00\x00\x00\x00\x00'  # clock 3s
        r.slot_raw(timestamp=None, data=clock_data)
        bus.publish.assert_called_with('sim_time_sec', 3)  # initially desired speed is None -> no cmd

        r.slot_desired_speed(timestamp=None, data=[0, 0])
        r.slot_raw(timestamp=None, data=clock_data)
        # ... asserting that the last call has been made in a particular way
        bus.publish.assert_called_with('cmd', b'cmd_vel 0.000000 0.000000')

        # after update from 3D desired speed only extended cmd_vel_3d should be used
        r.slot_desired_speed_3d(timestamp=None, data=[[1, 2, 3], [4, 5, 6]])
        r.slot_raw(timestamp=None, data=clock_data)
        bus.publish.assert_called_with('cmd', b'cmd_vel_3d 1.000000 2.000000 3.000000 4.000000 5.000000 6.000000')

    def test_publish_undefined_desired_speed(self):
        # for the drone it is important not to send any command until the very first desired speed is received
        pass

# vim: expandtab sw=4 ts=4
