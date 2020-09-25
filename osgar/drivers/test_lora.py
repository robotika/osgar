import unittest
import datetime
from unittest.mock import MagicMock, patch, call

from osgar.drivers.lora import LoRa, parse_lora_packet, split_lora_buffer, parse_my_cmd
from osgar.bus import Bus


class LoRaTest(unittest.TestCase):

    def test_parse_packet(self):
        addr, data = parse_lora_packet(b'4|alive\r\n')
        self.assertEqual(addr, [4])
        self.assertEqual(data, b'alive')
        addr, data = parse_lora_packet(b'4|1|2|pose(1, 2, 3)\r\n')
        self.assertEqual(addr, [4, 1, 2])
        self.assertEqual(data, b'pose(1, 2, 3)')

    def test_autodetect(self):
        SAMPLE_DATA = b'1|cmd=home\n'
        logger = MagicMock(write=MagicMock(return_value=datetime.timedelta()))
        bus = Bus(logger)
        c = LoRa(bus=bus.handle('lora'), config={'device_id': 4})
        tester = bus.handle('tester')
        tester.register('raw')
        bus.connect('tester.raw', 'lora.raw')
        c.start()
        tester.publish('raw', SAMPLE_DATA)
        c.request_stop()
        c.join()
        self.assertEqual(c.device_id, 4)

    def test_split_lora_buffer(self):
        self.assertEqual(split_lora_buffer(b'1|ble'), (b'1|ble', b''))
        self.assertEqual(split_lora_buffer(b'1|ble\n3|'), (b'3|', b'1|ble\n'))

    def test_invalid_packet(self):
        addr, data = parse_lora_packet(b'|pose(1, 2, 3)\r\n')
        self.assertIsNone(addr)
        self.assertEqual(data, b'|pose(1, 2, 3)\r\n')

    def test_autodetect_bug(self):
        logger = MagicMock()
        logger.write = MagicMock(return_value=datetime.timedelta(microseconds=9721))
        bus = Bus(logger)
        c = LoRa(bus=bus.handle('lora'), config={})  # force autodetection
        tester = bus.handle('tester')
        tester.register('raw')
        bus.connect('tester.raw', 'lora.raw')
        c.start()
        tester.publish('raw', b'4|alive\n')
        tester.publish('raw', b'4|alive-97')
        tester.publish('raw', b'21\n')
        c.request_stop()
        c.join()
        self.assertEqual(c.device_id, 4)

    def test_3rd_party_packet(self):
        # there are internal debug messages and also 3rd party packets, which we definetely
        # do not want to re-transmmit and process
        # In particular there was a bug that missing '|' generated empty addr list
        self.assertIsNone(parse_lora_packet(b'dhcps: send_off')[0])

        self.assertIsNone(parse_lora_packet(b'0|data')[0])
        self.assertIsNone(parse_lora_packet(b'7|data')[0])
        self.assertIsNone(parse_lora_packet(b'42|data')[0])

        # validate all addresses in chain
        self.assertIsNone(parse_lora_packet(b'4|7|data')[0])

    def test_parse_cmd(self):
        self.assertEqual(parse_my_cmd(4, b'4:GoHome:14151'), b'GoHome')
        self.assertEqual(parse_my_cmd(4, b'0:Pause:11'), b'Pause')  # 0 for ALL
        self.assertIsNone(parse_my_cmd(2, b'4:GoHome:14151'))
        self.assertIsNone(parse_my_cmd(2, b'nonsense'))

    def test_send_cmd(self):
        logger = MagicMock()
        bus = Bus(logger)
        logger.write = MagicMock(return_value=datetime.timedelta(microseconds=9721))
        c = LoRa(bus=bus.handle('lora'), config={'device_id':3})
        tester = bus.handle('tester')
        tester.register('raw')
        bus.connect('lora.cmd', 'tester.cmd')
        bus.connect('tester.raw', 'lora.raw')
        c.start()
        tester.publish('raw', b'1|3:GoHome:1234\n')
        c.request_stop()
        c.join()
        self.assertEqual(tester.listen()[2], b'GoHome')

    def test_control_center(self):
        logger = MagicMock()
        bus = Bus(logger)
        logger.write = MagicMock(return_value=datetime.timedelta(seconds=97))
        c = LoRa(bus=bus.handle('lora'), config={'device_id':1})
        app = bus.handle('app')
        app.register('cmd')
        tester = bus.handle('serial')
        tester.register('raw')
        bus.connect('app.cmd', 'lora.cmd')
        bus.connect('lora.raw', 'serial.raw')
        c.start()
        app.publish('cmd', [3, b'Pause'])
        c.request_stop()
        c.join()
        self.assertEqual(tester.listen()[2], b'alive\n')
        self.assertEqual(tester.listen()[2], b'3:Pause:97\n')

    def test_report_artifact(self):
        bus = MagicMock()
        c = LoRa(bus=bus, config={'device_id':1})
        with patch('osgar.node.Node.update') as p:
            p.return_value = 'raw'
            c.raw = b"3|['TYPE_BACKPACK', 3506, -18369, -752]\n"
            c.update()
            self.assertEqual(bus.method_calls,
                             [call.register('raw', 'cmd', 'robot_status', 'artf', 'artf_xyz'),
                              call.publish('artf', [3, ['TYPE_BACKPACK', 3506, -18369, -752]]),
                              call.publish('raw', b"3|['TYPE_BACKPACK', 3506, -18369, -752]\n")]
                             )

    def test_on_radio(self):
        # virtual world
        data = [b'A0F150L', b"['TYPE_RESCUE_RANDY', 15982, 104845, 3080]\n"]
        bus = MagicMock()
        dev = LoRa(bus=bus, config={'device_id':1})
        bus.reset_mock()
        dev.on_radio(data)
        bus.publish.assert_called()

# vim: expandtab sw=4 ts=4
