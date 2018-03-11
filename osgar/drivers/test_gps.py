import unittest

from drivers.gps import GPS, checksum, str2ms, INVALID_COORDINATES


class GPSTest(unittest.TestCase):

    def test_checksum(self):
        self.assertEqual(checksum(b'GNGGA,182433.10,5007.71882,N,01422.50467,E,1,05,6.09,305.1,M,44.3,M,,'), b'41')

    def test_str2ms(self):
        self.assertEqual(str2ms(b'5007.71882'), 180463129)

    def test_parse_line(self):
        line = b'$GNGGA,182433.10,5007.71882,N,01422.50467,E,1,05,6.09,305.1,M,44.3,M,,*41'
        p = GPS.parse_line(line)
        self.assertEqual((p['lon'], p['lat']), (51750280, 180463129))  # arc milliseconds (x, y) = (lon, lat)

    def test_split(self):
        buf = b'blabla$GPGGAsomething*12\r\n'
        self.assertEqual(GPS.split_buffer(buf), (b'\r\n', b'$GPGGAsomething*12'))

        self.assertEqual(GPS.split_buffer(b'$toofew*1'), (b'$toofew*1', b''))

    def test_parsing_old_GPGGA(self):
        line = b'$GPGGA,051852.000,5005.0244,N,01430.3360,E,1,06,3.8,253.1,M,45.4,M,,0000*58'
        p = GPS.parse_line(line)
        self.assertEqual((p['lon'], p['lat']), (52220160, 180301464))  # arc milliseconds (x, y) = (lon, lat)

    def test_split_with_binary_data(self):
        buf = b'\xb5b\x010\x04\x01\xf8n\x8a\x0e\x15\x04\x00\x00\x05\x02\r\x07&&@\x00S\xff\xff\xff\x02\x04\x10\x07$\xa5' \
            b'\x00\x00\x00\x00\x00\x00\x13\x05\x04\x04\x15\x00h\x00\x00\x00\x00\x00\x03\x06\r\x07\x1b\r#\x00\xee\xfa' \
            b'\xff\xff\x00\x0c\r\x07/*`\x00\x1d\x00\x00\x00\x04\x0e\r\x07"\x17\x0b\x01\x12\x01\x00\x00\n\x18\x0c\x07"' \
            b'\x07\x9e\x00\x8b\x00\x00\x00\x01\x19\r\x070S8\x00\xc8\xff\xff\xff\x0f\x1a\x04\x01\x00\x04\x1f\x01\x00\x00' \
            b'\x00\x00\r\x1d\r\x071C\xe3\x00\xcf\xff\xff\xff\x06\x1f\r\x07&).\x01\xa2\x03\x00\x00\x14 \x00\x07$\xa5\x00' \
            b'\x00\x00\x00\x00\x00\x0cA\x04\x07\x1c\x05\x1b\x00\x00\x00\x00\x00\x12G\x04\x01\x00\x02*\x01\x00\x00\x00\x00' \
            b'\x0bH\x04\x04\x13\x0bW\x01\x00\x00\x00\x00\x08I\r\x07\x1b\x1c=\x01]\x02\x00\x00\x11O\r\x07)\x1c\x88\x00\xdf' \
            b'\xfe\xff\xff\x10P\r\x07(VD\x01}\xff\xff\xff\tQ\r\x07\',)\x00\x1a\x02\x00\x00\x07R\r\x07*N\xb5\x00\xd7\xfe' \
            b'\xff\xff\x0eS\r\x07!\x19\xd0\x00\xca\x00\x00\x00\x83\x17\xb5b\x01\x03\x10\x00\xf8n\x8a\x0e\x03\xdd\x00\x08' \
            b'iy\x00\x00\xdd\xd9\x03\x00\x95($GNGGA,194535.40,5007.71276,N,01422.49206,E,1,12,0.80,284.6,M,44.3,M,,*42\r\n'
        buf, nmea = GPS.split_buffer(buf)
        self.assertEqual(nmea, b'$GNGGA,194535.40,5007.71276,N,01422.49206,E,1,12,0.80,284.6,M,44.3,M,,*42')

    def test_invalid_coordinates(self):
        self.assertEqual(GPS.parse_line(b'$GPGGA,053446.426,,,,,0,00,,,M,0.0,M,,0000*56'), INVALID_COORDINATES)

# vim: expandtab sw=4 ts=4
