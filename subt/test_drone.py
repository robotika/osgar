import datetime
import math
import unittest
from unittest.mock import MagicMock, patch, call

from osgar.bus import Bus
from subt.drone import Drone, MAX_ANGULAR


class DroneTest(unittest.TestCase):

    def test_usage(self):
        logger = MagicMock()
        bus = Bus(logger)
        logger.write = MagicMock(return_value=datetime.timedelta(microseconds=9721))
        c = Drone(bus=bus.handle('drone'), config={})
        tester = bus.handle('tester')
        tester.register('desired_speed')
        bus.connect('tester.desired_speed', 'drone.desired_speed')
        bus.connect('drone.desired_speed_3d', 'tester.desired_speed_3d')
        c.start()
        tester.publish('desired_speed', [1000, 9000])
        c.request_stop()
        c.join()
        self.assertEqual(tester.listen()[2], [[1.0, 0.0, 0.0], [0.0, 0.0, MAX_ANGULAR]])

# vim: expandtab sw=4 ts=4
