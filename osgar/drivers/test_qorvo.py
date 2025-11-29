import unittest
from osgar.drivers.qorvo import Qorvo

from unittest.mock import MagicMock, call


class QorvoTest(unittest.TestCase):

    def test_split_data(self):
        config = {}
        handler = MagicMock()
        uwb = Qorvo(config, bus=handler)
        self.assertEqual(uwb.split_data(b''), (None, b''))
        data = bytes.fromhex('400100410d00000000000000000000000000490100')
        self.assertEqual(uwb.split_data(data),
                         (b'@\x01\x00', b'A\r\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00I\x01\x00'))
        data = b'A\r\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00I\x01\x00'
        self.assertEqual(uwb.split_data(data),
                         (bytes.fromhex('410d00000000000000000000000000'), b'I\x01\x00'))
        self.assertEqual(uwb.split_data(b'I\x01\x00'),
                         (b'I\x01\x00', b''))

    def test_range_decoding(self):
        config = {}
        handler = MagicMock()
        uwb = Qorvo(config, bus=handler)
        handler.reset_mock()
        data = bytes.fromhex('400100410d00000000000000000000000000490100')  # no anchor
        uwb.on_raw(data)
        self.assertEqual(handler.publish.call_args, call('range', []))
        handler.reset_mock()
        data = b'I\x15\x017M\xdc\x02\x00\x00d\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00d'  # single anchor
        uwb.on_raw(data)
        self.assertEqual(handler.publish.call_args, call('range', [[19767, 732]]))

# vim: expandtab sw=4 ts=4
