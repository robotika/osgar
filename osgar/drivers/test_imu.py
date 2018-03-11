import unittest
from unittest.mock import MagicMock

from osgar.drivers.imu import parse_line, IMU
from osgar.drivers.bus import BusHandler


class IMUTest(unittest.TestCase):

    nmea_line = b'$VNYMR,-095.878,+008.933,-009.419,+00.1785,+00.1474,+00.5479,+00.252,-03.706,-03.874,-01.012502,-00.234810,+00.247165*66'

    def test_parse_line(self):
        # (yaw, pitch, roll), (magx, y, z), (accx, y, z), (gyrox, y, z)
        self.assertEqual(len(parse_line(self.nmea_line)), 4)

    def test_start_stop(self):
        config = {}
        logger = MagicMock()
        bus = BusHandler(logger, out={'orientation':[]}, name='imu')
        imu = IMU(config, bus=bus)
        imu.start()
        imu.request_stop()
        imu.join()

    def test_processing(self):
        config = {}
        logger = MagicMock()
        robot_bus = BusHandler(logger, out={}, name='robot')
        bus = BusHandler(logger, out={'orientation':[(robot_bus.queue, 'orientation')]}, name='imu')
        imu = IMU(config, bus=bus)
        imu.start()
        imu.bus.queue.put((123, 'raw', self.nmea_line))
        data = robot_bus.listen()
        imu.request_stop()
        imu.join()

# vim: expandtab sw=4 ts=4
