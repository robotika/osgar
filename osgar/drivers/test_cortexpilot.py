import unittest
from unittest.mock import MagicMock, call

from osgar.drivers.cortexpilot import Cortexpilot
from osgar.bus import BusHandler

SAMPLE_DATA = bytes.fromhex(
        '000259010d0000000040952c416666be410000000000000000000000800000000000000' +
        '00000006fe702008013464840dd6e1db143f60900805f43ed44413d63e3ac3c3386edbb' +
        'aeb17f3f5d663943008037440000c040002079461ab754bd9c137d3cff149dbbf37345b' +
        'ed29ec8bbb763eabebf8304000000000000000000000000000000000000000000000000' +
        '00000000000000000000000000000000000000000000000000000000000000000000000' +
        '00000000000000000000000000000000000000000000000000000000000000000000000' +
        '00000000000000000000000000000000000000000000000000000000000000000000000' +
        '00000000000000000000000000000000000000000000000000000000000000000000000' +
        '00000000000000000000000000000000000000000000000000000000000000000000000' +
        '00000000000000000000000000000000000000000000000000000000000000000000000' +
        '00000000000000000000000000000000000000000000000000000000000000000000000' +
        '00000000000000000000000000000000000000000000000000000000000000000000000' +
        '00000000000000000000000000000000000000000000000000000000000000000000000' +
        '00000000000000000000000000000000000000000000000000000000000000000000000' +
        '00000000000000000000000000000000000000000000000000000000000000000000000' +
        '00000000000000000000000000000000000000000000000000000000000000000000000' +
        '000000000000000000000000000000000000000000000000000000000000000000000047')


class CortextpilotTest(unittest.TestCase):

    def test_usage(self):
        q = MagicMock()
        logger = MagicMock()
        logger.write = MagicMock(return_value=135)
        bus = BusHandler(logger=logger,
                out={'raw': [(q, 'raw')], 'encoders': [], 'emergency_stop': [],
                     'pose2d': [], 'buttons': []})
        robot = Cortexpilot(config={}, bus=bus)
        robot.start()
        robot.request_stop()
        robot.join()
        # at first request firmware version
        q.put.assert_called_once_with((135, 'raw', 
            b'\x00\x00\x03\x01\x01\xfb'))

    def test_2nd_loop(self):
        q = MagicMock()
        logger = MagicMock()
        logger.write = MagicMock(return_value=135)
        bus = BusHandler(logger=logger,
                out={'raw': [(q, 'raw')], 'encoders': [], 'emergency_stop': [],
                     'pose2d': [], 'buttons': []})
        robot = Cortexpilot(config={}, bus=bus)
        bus.queue.put((123, 'raw', b'\x00\x00\x10\x01\x01Robik V4.0.2\x00\x8f'))
        robot.start()
        robot.request_stop()
        robot.join()

        self.assertEqual(q.put.call_args_list, [
            call((135, 'raw', b'\x00\x00\x03\x01\x01\xfb')),  # request version
            call((135, 'raw', bytes.fromhex('00000f010d00000000000000804000000023'))) # cmd
            ])

    def test_create_packet(self):
        robot = Cortexpilot(config={}, bus=None)
        packet = robot.create_packet()
        self.assertEqual(len(packet), 3 + 15)
        self.assertEqual(sum(packet) % 256, 0)
        self.assertEqual(packet[-1], 0x23)

        robot.desired_speed = 0.5
        robot.lidar_valid = True  # otherwise the speed will be reset to zero
        packet = robot.create_packet()
        self.assertEqual(len(packet), 3 + 15)
        self.assertEqual(sum(packet) % 256, 0)
        self.assertEqual(packet[-1], 0x81)

        # test packet checksum
        robot.desired_speed = -0.12314114151
        packet = robot.create_packet()

    def test_get_packet(self):
        robot = Cortexpilot(config={}, bus=None)
        packet = robot.get_packet()
        self.assertIsNone(packet)

        robot._buf = SAMPLE_DATA
        packet = robot.get_packet()
        self.assertIsNotNone(packet)
        self.assertEqual(len(packet), len(SAMPLE_DATA))
        self.assertEqual(len(robot._buf), 0)

        packet = robot.get_packet()
        self.assertIsNone(packet)

    def test_parse_packet(self):
        robot = Cortexpilot(config={}, bus=MagicMock())
        robot.parse_packet(SAMPLE_DATA)
        self.assertEqual(robot.flags, 0x0)
        self.assertAlmostEqual(robot.voltage, 10.78643798)

        # trigger pose update
        robot.parse_packet(SAMPLE_DATA)

# vim: expandtab sw=4 ts=4
