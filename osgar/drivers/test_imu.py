import unittest
from unittest.mock import MagicMock

from osgar.drivers.imu import parse_line, IMU
from osgar.bus import Bus


class IMUTest(unittest.TestCase):

    nmea_line = b'$VNYMR,-095.878,+008.933,-009.419,+00.1785,+00.1474,+00.5479,+00.252,-03.706,-03.874,-01.012502,-00.234810,+00.247165*66'
    orientation = [[-95.878, 8.933, -9.419], [0.1785, 0.1474, 0.5479], [0.252, -3.706, -3.874], [-1.012502, -0.23481, 0.247165]]

    def test_parse_line(self):
        # (yaw, pitch, roll), (magx, y, z), (accx, y, z), (gyrox, y, z)
        self.assertEqual(len(parse_line(self.nmea_line)), 4)

    def test_start_stop(self):
        config = {}
        logger = MagicMock()
        bus = Bus(logger)
        imu = IMU(config, bus=bus.handle('imu'))
        imu.start()
        imu.request_stop()
        imu.join()

    def test_processing(self):
        config = {}
        logger = MagicMock()
        logger.write = MagicMock(return_value=135)
        bus = Bus(logger)
        #robot_bus = BusHandler(logger, out={}, name='robot')
        #bus = BusHandler(logger,
        #                 out={'orientation':[(robot_bus.queue, 'orientation')], 'rotation':[]},
        #                 name='imu')
        imu = IMU(config, bus=bus.handle('imu'))
        tester = bus.handle('tester')
        tester.register('raw')
        bus.connect('tester.raw', 'imu.raw')
        bus.connect('imu.orientation', 'tester.orientation')
        imu.start()
        tester.publish('raw', self.nmea_line)
        imu.request_stop()
        imu.join()
        tester.shutdown()
        self.assertEqual(tester.listen(), (135, 'orientation', self.orientation))

# vim: expandtab sw=4 ts=4
