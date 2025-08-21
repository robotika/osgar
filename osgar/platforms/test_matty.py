import datetime
import unittest
from unittest.mock import MagicMock, call
import math
from datetime import timedelta

from osgar.platforms.matty import Matty, add_esc_chars, SYNC, ESC, remove_esc_chars, FRONT_REAR_AXIS_DISTANCE


class MattyTest(unittest.TestCase):

    def test_usage(self):
        bus = MagicMock()
        robot = Matty(bus=bus, config={})
        robot.on_tick(None)
        bus.publish.assert_called_with('esp_data', bytes.fromhex('55 03 03 50 01 a9'))  # P - request GPS
#        bus.publish.assert_called_with('esp_data', bytes.fromhex('55 06 01 54 64 00 E8 03 56 A9'))  # set sending msg

    def test_crc(self):
        bus = MagicMock()
        robot = Matty(bus=bus, config={})
        # packet 0 is not acknowledged
#        with self.assertRaises(AssertionError):
        # changed to warnin only
        robot.on_esp_data(bytes.fromhex('55 02 00 4e b0'))

        # ACK and proper counter
        robot.counter = 1
        robot.on_esp_data(bytes.fromhex('55020141bc'))

    def test_send_esp(self):
        bus = MagicMock()
        robot = Matty(bus=bus, config={})
        robot.send_esp(b'S')
        bus.publish.assert_called_with('esp_data', bytes().fromhex('55020153aa'))

    def test_send_esp(self):
        bus = MagicMock()
        robot = Matty(bus=bus, config={})
        robot.send_speed()
        bus.publish.assert_called_with('esp_data', b'U\x06\x01G\x00\x00\x00\x00\xb2')

    def test_add_esc_chars(self):
        self.assertEqual(add_esc_chars(bytes([1, 2, 3, 4])), bytes([1, 2, 3, 4]))
        self.assertEqual(add_esc_chars(bytes([1, SYNC, 3, 4])), bytes([1, ESC, 0xFF & (~SYNC), 3, 4]))
        self.assertEqual(add_esc_chars(bytes([1, 2, ESC, 4])), bytes([1, 2, ESC, 0xFF & (~ESC), 4]))

    def test_long_data(self):
        bus = MagicMock()
        robot = Matty(bus=bus, config={})
        robot.time = datetime.timedelta(0.1)
        robot.on_esp_data(bytes.fromhex('5514104901025c1c7b000000000065001700dd05ea0056aa5514114901025c1c7b000000000065001700dd05ea00545514124901025c1c7b000000000065001700dd05ea00535514134901025c1c7b000000000065001700dd05ea00525514144901025c1c7b000000000065001700dd05ea00515514154901025c1c7b000000000065001700dd05ea00505514164901025c1c7b000000000065001700dd05ea004f5514174901025c1c7b000000000065001700dd05ea004e5514184901025c1c7b000000000065001700dd05ea004d5514194901025c1c7b000000000065001700dd05ea004c55141a4901025c1c7b000000000065001700dd05ea004b55141b4901025c1c7b000000000065001700dd05ea004a55141c4901025c1c7b000000000065001700dd05ea004955141d4901025c1c7b000000000065001700dd05ea004855141e4901025c1c7b000000000065001700dd05ea004755141f4901025c1c7b000000000065001700dd05ea00465514204901025c1c7b000000000065001700dd05ea00455514214901025c1c7b000000000065001700dd05ea00445514224901025c1c7b000000000065001700dd05ea00435514234901025c1c7b000000000065001700dd05ea00425514244901025c1c7b000000000065001700dd05ea00415514254901025c1c7b000000000065001700dd05ea00405514264901025c1c7b000000000065001700dd05ea003f5514274901025c1c7b000000000065001700dd05ea003e5514284901025c1c7b000000000065001700dd05ea003d5514294901025c1c7b000000000065001700dd05ea003c55142a4901025c1c7b000000000065001700dd05ea003b55142b4901025c1c7b000000000065001700dd05ea003a55142c4901025c1c7b000000000065001700dd05ea003955142d4901025c1c7b000000000065001700dd05ea003855142e4901025c1c7b000000000065001700dd05ea003755142f4901025c1c7b000000000065001700dd05ea00365514304901025c1c7b000000000065001700dd05ea00355514314901025c1c7b000000000065001700dd05ea00345514324901025c1c7b000000000065001700dd05ea00335514334901025c1c7b000000000065001700dd05ea00325514344901025c1c7b000000000065001700dd05ea003155020241bb5514354901025c1c7b000000000065001700dd05ea00305514364901025c1c7b000000000065001700dd05ea002f5514374901025c1c7b000000000065001700dd05ea002e5514384901025c1c7b000000000065001700dd05ea002d5514394901025c1c7b000000000065001700dd05ea002c55143a4901025c1c7b000000000065001700dd05ea002b55143b4901025c1c7b000000000065001700dd05ea002a55143c4901025c1c7b000000000065001700dd05ea002955143d4901025c1c7b000000000065001700dd05ea002855143e4901025c1c7b000000000065001700dd05ea002755143f4901025c1c7b000000000065001700dd05ea00265514404901025c1c7b000000000065001700dd05ea00255514414901025c1c7b000000000065001700dd05ea00245514424901025c1c7b000000000065001700dd05ea00235514434901025c1c7b000000000065001700dd05ea00225514444901025c1c7b000000000065001700dd05ea00215514454901025c1c7b000000000065001700dd05ea00205514464901025c1c7b000000000065001700dd05ea001f5514474901025c1c7b000000000065001700dd05ea001e5514484901025c1c7b000000000065001700dd05ea001d5514494901025c1c7b000000000065001700dd05ea001c55144a4901025c1c7b000000000065001700dd05ea001b55144b4901025c1c7b000000000065001700dd05ea001a55144c4901025c1c7b000000000065001700dd05ea001955144d4901025c1c7b000000000065001700dd05ea001855144e4901025c1c7b000000000065001700dd05ea001755144f4901025c1c7b000000000065001700dd05ea00165514504901025c1c7b000000000065001700dd05ea00155514514901025c1c7b000000000065001700dd05ea00145514524901025c1c7b000000000065001700dd05ea00135514534901025c1c7b000000000065001700dd05ea00125514544901025c1c7b000000000065001700dd05ea0011551456aa4901025c1c7b000000000065001700dd05ea0010551456a94901025c1c7b000000000065001700dd05ea000f5514574901025c1c7b000000000065001700dd05ea000e5514584901025c1c7b00'))

    def test_remove_esc_chars(self):
        self.assertEqual(remove_esc_chars(bytes([1, 2, 3, 4])), bytes([1, 2, 3, 4]))
        self.assertEqual(remove_esc_chars(bytes([1, ESC, 0xFF & (~SYNC), 3, 4])), bytes([1, SYNC, 3, 4]))
        self.assertEqual(remove_esc_chars(bytes([1, 2, ESC, 0xFF & (~ESC), 4])), bytes([1, 2, ESC, 4]))

    def test_parse_odometry(self):
        bus = MagicMock()
        robot = Matty(bus=bus, config={})
        robot.time = datetime.timedelta(0.1)
        speed, steering = robot.parse_odometry(bytes.fromhex('104901025c1c7b000000000065001700dd05ea00'))
        self.assertAlmostEqual(speed, 0.0)
        self.assertAlmostEqual(steering, math.radians(0))

    def test_pose2d(self):
        bus = MagicMock()
        robot = Matty(bus=bus, config={})
        robot.publish_pose2d(0.5, 0.0)
        self.assertAlmostEqual(robot.pose[0], 0.05)
        self.assertAlmostEqual(robot.pose[1], 0.00)
        self.assertAlmostEqual(robot.pose[2], 0.00)

        # turn
        robot.publish_pose2d(0.5, math.radians(90))
        self.assertAlmostEqual(robot.pose[0], 0.1)  # simplified interpolation
        self.assertAlmostEqual(robot.pose[1], 0.00)
        self.assertAlmostEqual(robot.pose[2], 0.5 * 0.1 / (FRONT_REAR_AXIS_DISTANCE/2))

        robot.publish_pose2d(0.5, math.radians(90))
        self.assertAlmostEqual(robot.pose[0], 0.14757839740240863)
        self.assertAlmostEqual(robot.pose[1], 0.015371925729019041)
        self.assertAlmostEqual(robot.pose[2], 2 * 0.5 * 0.1 / (FRONT_REAR_AXIS_DISTANCE/2))

    def test_multiple_messages(self):
        # bug causing serious delays
        bus = MagicMock()
        robot = Matty(bus=bus, config={})
        robot.process_esp_packet = MagicMock()
        robot.on_esp_data(b'U\x14EI\x00\x02`/\x00\x00\x00\x00N\x01^\x06W\x05&\x07\x88\x07\x02U\x02\x01A\xbcU\x02\x02A\xbbU\x02\x03A\xba')
        robot.process_esp_packet.assert_called()
        self.assertEqual(len(robot.process_esp_packet.mock_calls), 3)

    def test_emergency_stop(self):
        # make sure that "emergency_stop" is correctly published
        bus = MagicMock()
        robot = Matty(bus=bus, config={})
        data = b'\x8bI\xb1\x02\x94\x03\xff\xff\x00\x00\x00\x00\x81\x1a\x81\x1a\x81\x1a\x81\x1a'
        bus.reset_mock()
        robot.parse_odometry(data)
        bus.publish.assert_called()
        self.assertEqual(len(bus.publish.mock_calls), 5)  # 2x bumpers, speed command, emergency_stop status, joint_angle
        bus.publish.assert_called_with('joint_angle', [0])
        self.assertEqual(bus.publish.mock_calls[-2], call('esp_data', b'U\x06\x01G\x00\x00\x00\x00\xb2'))
        bus.reset_mock()
        robot.parse_odometry(data)
        # same data, no change - only 'joint_angle'
        bus.publish.assert_called()
        bus.publish.assert_called_with('joint_angle', [0])

        data = b'|I\xb0\x02\\0\xff\xff\x00\x00Y\xff\x00\x00\x00\x00\x00\x00\x00\x00'
        bus.reset_mock()
        robot.parse_odometry(data)
        bus.publish.assert_called()
        self.assertEqual(len(bus.publish.mock_calls), 2)  # only emergency_stop status change + joint_angle
        self.assertEqual(bus.publish.mock_calls[0], call('emergency_stop', False))

        data = b'\x00I1\x02\x0c\x00\xff\xff\x00\x00\xab\x04\x00\x00\x00\x00\x00\x00\x00\x00'
        bus.reset_mock()
        robot.parse_odometry(data)
        bus.publish.assert_called()
        self.assertEqual(len(bus.publish.mock_calls), 2)  # only emergency_stop status change
        self.assertEqual(bus.publish.mock_calls[0], call('emergency_stop', True))

    def test_ver8_with_imu(self):
        bus = MagicMock()
        robot = Matty(bus=bus, config={})
        bus.reset_mock()
        robot.process_esp_packet(bytes.fromhex('01490002982b0000fcff9e00cbff47ff65ff4fff94fe3d00a3d5'))
        bus.publish.assert_called()
        self.assertEqual(bus.publish.mock_calls[0], call('rpy', [-364, 61, -10845]))

    def test_east_direction(self):
        # well, almost east
        data = bytes.fromhex('d4498002e02f0a0000004600cb2f7e30842fac2ed0007d01581c')
        bus = MagicMock()
        robot = Matty(bus=bus, config={})
        bus.reset_mock()
        robot.parse_odometry(data)
        bus.publish.assert_called()
        self.assertEqual(bus.publish.mock_calls[0], call('rpy', [208, 381, 7256]))
        self.assertEqual(bus.publish.mock_calls[1], call('rotation', [9000-7256, -381, 208]))
        self.assertEqual(bus.publish.mock_calls[2], call('orientation',
                [0.022969623697164748, -0.030102545599598646, 0.1520934996010254, 0.9876405219080208]))
