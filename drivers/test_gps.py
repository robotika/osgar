import unittest

from drivers.gps import GPS, checksum, str2ms


class GPSTest(unittest.TestCase):

    def test_checksum(self):
        self.assertEqual(checksum(b'GNGGA,182433.10,5007.71882,N,01422.50467,E,1,05,6.09,305.1,M,44.3,M,,'), b'41')

    def test_str2ms(self):
        self.assertEqual(str2ms(b'5007.71882'), 180463129)

    def test_parse_line(self):
        line = b'$GNGGA,182433.10,5007.71882,N,01422.50467,E,1,05,6.09,305.1,M,44.3,M,,*41'
        self.assertEqual(GPS.parse_line(line), (51750280, 180463129))  # arc milliseconds (x, y) = (lon, lat)

    def test_split(self):
        buf = b'blabla$something*12'
        self.assertEqual(GPS.split_buffer(buf), (b'', b'$something*12'))

        self.assertEqual(GPS.split_buffer(b'$toofew*1'), (b'$toofew*1', b''))

    def test_parsing_old_GPGGA(self):
        line = b'$GPGGA,051852.000,5005.0244,N,01430.3360,E,1,06,3.8,253.1,M,45.4,M,,0000*58'
        self.assertEqual(GPS.parse_line(line), (52220160, 180301464))  # arc milliseconds (x, y) = (lon, lat)

# vim: expandtab sw=4 ts=4
