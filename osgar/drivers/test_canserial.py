import unittest
from unittest.mock import MagicMock

from osgar.drivers.canserial import CANSerial, CAN_packet


class CANSerialTest(unittest.TestCase):

    def test_split_buffer(self):
        self.assertEqual(CANSerial.split_buffer(b''), (b'', b''))
        self.assertEqual(CANSerial.split_buffer(b'\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xfe\x10'), (b'', b'\xfe\x10'))
        data = b'\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xfeW\xfe0@h\x9e\x01i\x01\xf7\x01\x18\x00'
        self.assertEqual(CANSerial.split_buffer(data), (b'\xfe0@h\x9e\x01i\x01\xf7\x01\x18\x00', b'\xfeW'))

        data = b'0@h\x9e\x01i\x01\xf7\x01\x18\x00'
        self.assertEqual(CANSerial.split_buffer(data), (b'h\x9e\x01i\x01\xf7\x01\x18\x00', b'0@'))

    def test_can_packet(self):
        self.assertEqual(CAN_packet(0x400, [0, 0]), b'\x80\x02\x00\x00')

    def test_can_bridge_initialization(self):
        bus = MagicMock()
        can = CANSerial(config={}, bus=bus)
        self.assertFalse(can.can_bridge_initialized)

        for __ in can.process_gen(b'\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xfe\x10'):
            pass
        self.assertTrue(can.can_bridge_initialized)

        bus.publish.assert_called_with('raw', b'\xfe1')

    def test_invalid_config(self):
        bus = MagicMock()
        config =  {"speed": 123, "canopen":True}
        with self.assertRaises(ValueError) as err:
            can = CANSerial(config=config, bus=bus)
        self.assertEqual(str(err.exception), "unsupported speed: 123\n" +
            "Use:['10k', '20k', '50k', '125k', '250k', '500k', '800k', '1M']")

    def test_reset_modules(self):
        bus = MagicMock()
        can = CANSerial(config={}, bus=bus)

        self.assertEqual(can.modules_for_restart, set())
        can.check_and_restart_modules(2, 127)
        bus.publish.assert_called_with('raw', b'\x00\x02\x81\x02')
        self.assertEqual(can.modules_for_restart, set([2]))

        can.check_and_restart_modules(2, 127)
        bus.publish.assert_called_with('raw', b'\x00\x02\x01\x02')

        can.check_and_restart_modules(2, 5)
        self.assertEqual(can.modules_for_restart, set())

# vim: expandtab sw=4 ts=4
