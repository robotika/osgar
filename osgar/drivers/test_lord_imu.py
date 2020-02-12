import unittest
import datetime
from unittest.mock import MagicMock

from osgar.drivers.lord_imu import get_packet, verify_checksum, parse_packet, LordIMU
from osgar.bus import Bus


SAMPLE_DATA = b'\x0e\x06=\xea\x01\x19>\xc3R\xd2>\xdb?V\x1e\x93ue\x80*\x0e\x04<\x87C\x97\xbdF%\xd4\xbf\x7f\xb4\x14\x0e\x05\xba.\xd6S\xba)b\xeb\xb9\xf6\xf7\x80\x0e\x06=\xeaD\x10>\xc3\\\xd5>\xdb4\x1d\xda\x04ue\x80*\x0e\x04<\x9d\x1c\xa3\xbdS\x19\xd9\xbf\x7f\xbc^\x0e\x05\xbas\xd7W\xb9\x15\x03\xa0\xbaD\xffc\x0e\x06=\xea'

SAMPLE_GPS_DATA = bytes.fromhex('756581432C034048F4AC14C660A2402C66410EBE08E44071A36872B020C5406DB35C28F5C2904008E5604062F1AA001F0F0807E30610090D11000000000003080B000900000007D716')


class LordIMUTest(unittest.TestCase):

    def test_parse_packet(self):
        self.assertEqual(get_packet(b''), (None, b''))

        buf = SAMPLE_DATA
        packet, buf = get_packet(buf)
        self.assertTrue(packet.startswith(b'\x75\x65'), packet)
        self.assertEqual(len(packet), 42 + 4 + 2)

        acc, gyro, mag, quat, gps = parse_packet(packet)

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
        logger = MagicMock()
        bus = Bus(logger)
        tester = bus.handle('tester')
        tester.register('raw')
        imu = LordIMU(config={}, bus=bus.handle('lord'))
        bus.connect('tester.raw', 'lord.raw')
        imu.start()
        tester.publish('raw', SAMPLE_DATA)
        imu.request_stop()
        imu.join()
        self.assertEqual(imu.raw, SAMPLE_DATA)
#        q.put.assert_called_once_with((135, 'raw',
#            b'\x00\x00\x03\x01\x01\xfb'))

    def test_parse_gps(self):
        verify_checksum(SAMPLE_GPS_DATA)
        __, __, __, __, gps = parse_packet(SAMPLE_GPS_DATA)
        self.assertEqual(gps, ((49.9115015, 14.199715099999999), 9,
                               datetime.datetime(2019, 6, 16, 9, 13, 17)))

    def test_index_error(self):
        """
           bug - used to raise Index Error:
File "M:\git\osgar\osgar\drivers\lord_imu.py", line 28, in get_packet
    packet_size = buf[i + 3] + 4 + 2  # + header size + checksum
IndexError: index out of range
"""
        packet, buf = get_packet(b'ue')
        self.assertIsNone(packet)
        self.assertEqual(buf, b'ue')

    def test_invalid_checksum(self):
        raw_data = bytes.fromhex('75658252100d3d3302803c68b2e03cee54000001100e3bb32f1f3b0ed8403a637b3e00010e14000000000'
                                 '00000000001000214033c7a802ebf4e2607bf17b2a78ae04161ef4e0001100ebd08efc83ebe409a3cb8df380001')
        self.assertFalse(verify_checksum(raw_data))

        packet, new_buf = get_packet(raw_data + bytes.fromhex('1234'))
        self.assertIsNone(packet)
        self.assertEqual(new_buf, bytes.fromhex('1234'))

# vim: expandtab sw=4 ts=4
