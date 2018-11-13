import unittest
from unittest.mock import MagicMock, call

from osgar.drivers.eduro import Eduro, sint32_diff
from osgar.drivers.canserial import CAN_packet
from osgar.bus import BusHandler


class EduroTest(unittest.TestCase):

    def test_sync(self):
        q = MagicMock()
        logger = MagicMock()
        logger.write = MagicMock(return_value=135)
        bus = BusHandler(logger=logger,
                out={'can': [], 'encoders': [], 'emergency_stop': [],
                     'pose2d': [(q, 'pose2d'),], 'buttons': []})
        eduro = Eduro(config={}, bus=bus)
        sync = CAN_packet(0x80, [])
        bus.queue.put((123, 'can', sync))
        bus.shutdown()
        eduro.run()
        q.put.assert_called_once_with((135, 'pose2d', [0, 0, 0]))

    def test_buttons(self):
        # is there a simpler way without starting the Thread??
        q = MagicMock()
        logger = MagicMock()
        logger.write = MagicMock(return_value=42)
        bus = BusHandler(logger=logger,
                out={'can': [], 'encoders': [], 'emergency_stop': [],
                     'pose2d': [], 'buttons': [(q, 'buttons')]})
        eduro = Eduro(config={}, bus=bus)
        buttons_msg = CAN_packet(0x28A, [0, 0])
        bus.queue.put((42, 'can', buttons_msg))
        bus.shutdown()
        eduro.run()
        q.put.assert_called_once_with((42, 'buttons', {'blue_selected': True, 'cable_in': False}))

    def test_encoders_overflow(self):
        q = MagicMock()
        logger = MagicMock()
        logger.write = MagicMock(return_value=22)
        bus = BusHandler(logger=logger,
                out={'can': [], 'encoders': [(q, 'encoders')], 'emergency_stop': [],
                     'pose2d': [], 'buttons': []})
        eduro = Eduro(config={}, bus=bus)
        sync = CAN_packet(0x80, [])

        enc_left = CAN_packet(0x181, [0xff, 0xff, 0xff, 0x7f])
        bus.queue.put((42, 'can', enc_left))
        bus.queue.put((123, 'can', sync))

        enc_left = CAN_packet(0x181, [0x01, 0x00, 0x00, 0x80])
        bus.queue.put((44, 'can', enc_left))
        sync = CAN_packet(0x80, [])
        bus.queue.put((123, 'can', sync))

        bus.shutdown()
        eduro.run()
        self.assertEqual(q.put.call_args_list, [call((22, 'encoders', [0, 0])),
                                                call((22, 'encoders', [2, 0]))])

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
        bus.publish.assert_called_once_with('can', b'@$\xa9\x05\xa0\x0f')

# vim: expandtab sw=4 ts=4
