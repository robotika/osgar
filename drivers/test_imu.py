import unittest

from drivers.imu import parse_line

class IMUTest(unittest.TestCase):

    def test_parse_line(self):
        # (yaw, pitch, roll), (magx, y, z), (accx, y, z), (gyrox, y, z)
        line = b'$VNYMR,-095.878,+008.933,-009.419,+00.1785,+00.1474,+00.5479,+00.252,-03.706,-03.874,-01.012502,-00.234810,+00.247165*66'
        self.assertEqual(len(parse_line(line)), 4)
#        self.assertIsNone()
