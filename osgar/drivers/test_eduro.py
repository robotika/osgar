import unittest
from unittest.mock import MagicMock

from osgar.drivers.eduro import Eduro, sint32_diff, CAN_triplet
from osgar.bus import Bus


class EduroTest(unittest.TestCase):

    def test_sync(self):
        logger = MagicMock()
        logger.write = MagicMock(return_value=135)
        bus = Bus(logger)
        eduro = Eduro(config={}, bus=bus.handle('eduro'))
        tester = bus.handle('tester')
        tester.register('can')
        bus.connect('tester.can', 'eduro.can')
        bus.connect('eduro.pose2d', 'tester.pose2d')
        sync = CAN_triplet(0x80, [])
        tester.publish('can', sync)
        eduro.request_stop()
        eduro.run()
        tester.shutdown()
        self.assertEqual(tester.listen(), (135, 'pose2d', [0, 0, 0]))

    def test_buttons(self):
        logger = MagicMock()
        logger.write = MagicMock(return_value=42)
        bus = Bus(logger)
        eduro = Eduro(config={}, bus=bus.handle('eduro'))
        tester = bus.handle('tester')
        tester.register('can')
        bus.connect('eduro.buttons', 'tester.buttons')
        bus.connect('tester.can', 'eduro.can')
        tester.publish('can', CAN_triplet(0x28A, [0, 0]))
        eduro.request_stop()
        eduro.run()
        tester.shutdown()
        self.assertEqual(tester.listen(), (42, 'buttons', {'blue_selected': True, 'cable_in': False}))

    def test_encoders_overflow(self):
        logger = MagicMock()
        logger.write = MagicMock(return_value=22)
        bus = Bus(logger)
        eduro = Eduro(config={}, bus=bus.handle('eduro'))
        tester = bus.handle('tester')
        tester.register('can')
        bus.connect('eduro.encoders', 'tester.encoders')
        bus.connect('tester.can', 'eduro.can')

        sync = CAN_triplet(0x80, [])

        enc_left = CAN_triplet(0x181, [0xff, 0xff, 0xff, 0x7f])
        tester.publish('can', enc_left)
        tester.publish('can', sync)

        enc_left = CAN_triplet(0x181, [0x01, 0x00, 0x00, 0x80])
        tester.publish('can', enc_left)
        tester.publish('can', sync)
        eduro.request_stop()
        eduro.run()
        tester.shutdown()
        self.assertEqual(tester.listen(), (22, 'encoders', [0, 0]))
        self.assertEqual(tester.listen(), (22, 'encoders', [2, 0]))

    def test_sint32_diff(self):
        self.assertEqual(sint32_diff(-5, 7), -12)
        self.assertEqual(sint32_diff(-0x7FFFFFFF, 0x7FFFFFFF), 2)
        self.assertEqual(sint32_diff(0x7FFFFFFF, -0x7FFFFFFF), -2)

    def test_float_speed_bug(self):
        bus = MagicMock()
        eduro = Eduro(config={}, bus=bus)
        # set internal values before crash
        eduro.desired_speed = 0.71
        eduro.desired_angular_speed = 1.6952383024620923
        eduro._rampLastLeft = 1417
        eduro._rampLastRight = 3891
        eduro.send_speed()
        bus.publish.assert_called_once_with('can', [0x201, b'\xa9\x05\xa0\x0f', 0])

# vim: expandtab sw=4 ts=4
