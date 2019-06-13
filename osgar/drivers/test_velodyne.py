import unittest
from unittest.mock import MagicMock

from osgar.drivers.velodyne import parse_packet, Velodyne
from osgar.bus import BusHandler


SAMPLE_DATA = b''  # TODO


class VelodyneTest(unittest.TestCase):

    def test_parse_packet(self):
        self.assertEqual(parse_packet(b''), [])


    def test_node(self):
        config = {}
        logger = MagicMock()
        robot_bus = BusHandler(logger, out={}, name='robot')
        bus = BusHandler(logger,
                         out={'xyz':[(robot_bus.queue, 'xyz')]},
                         name='velodyne')
        imu = Velodyne(config, bus=bus)
        imu.start()
        imu.bus.queue.put((123, 'raw', SAMPLE_DATA))
        imu.request_stop()
        imu.join()

# vim: expandtab sw=4 ts=4
