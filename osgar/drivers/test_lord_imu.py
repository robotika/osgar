import unittest
from unittest.mock import MagicMock

from osgar.drivers.lord_imu import get_packet, verify_checksum, parse_packet, LordIMU
from osgar.bus import BusHandler


SAMPLE_DATA = b'\x0e\x06=\xea\x01\x19>\xc3R\xd2>\xdb?V\x1e\x93ue\x80*\x0e\x04<\x87C\x97\xbdF%\xd4\xbf\x7f\xb4\x14\x0e\x05\xba.\xd6S\xba)b\xeb\xb9\xf6\xf7\x80\x0e\x06=\xeaD\x10>\xc3\\\xd5>\xdb4\x1d\xda\x04ue\x80*\x0e\x04<\x9d\x1c\xa3\xbdS\x19\xd9\xbf\x7f\xbc^\x0e\x05\xbas\xd7W\xb9\x15\x03\xa0\xbaD\xffc\x0e\x06=\xea'


class LordIMUTest(unittest.TestCase):

    def test_parse_packet(self):
        self.assertEqual(get_packet(b''), (None, b''))

        buf = SAMPLE_DATA
        packet, buf = get_packet(buf)
        self.assertTrue(packet.startswith(b'\x75\x65'), packet)
        self.assertEqual(len(packet), 42 + 4 + 2)

        acc, gyro, mag, quat = parse_packet(packet)

    def test_checksum(self):
        packet = bytes.fromhex("7565 0C20 0D08 0103 1200 0A04 000A 0500 0A13 0A01 0511 000A 1000 0A01 000A 0200 0A03 000A D43D")
        verify_checksum(packet)

        ping = bytes.fromhex('7565 0102 0201 E0C6')
        verify_checksum(ping)
        #ping 0xe4c6
        #replay: 7565 0104 04F1 0100 D56A
        # set to idle: 7565 0102 0202 E1C7
        # ping         7565 0102 0201 E0C6

    def test_node(self):
        config = {}
#        q = MagicMock()
        logger = MagicMock()
        robot_bus = BusHandler(logger, out={}, name='robot')
        bus = BusHandler(logger,
                         out={'orientation':[(robot_bus.queue, 'orientation')], 'rotation':[]},
                         name='imu')
        imu = LordIMU(config, bus=bus)
        imu.start()
        imu.bus.queue.put((123, 'raw', SAMPLE_DATA))
        imu.request_stop()
        imu.join()
        self.assertEqual(imu.raw, SAMPLE_DATA)
#        q.put.assert_called_once_with((135, 'raw', 
#            b'\x00\x00\x03\x01\x01\xfb'))

# vim: expandtab sw=4 ts=4
