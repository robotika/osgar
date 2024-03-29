import datetime
import math
import unittest
from unittest.mock import MagicMock, patch, call

from osgar.bus import Bus
from subt.drone import Drone, MAX_ANGULAR, altitude_from_pressure


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

    def test_altitude_from_pressure(self):
        self.assertEqual(altitude_from_pressure(101323.49836826918), 0.12500215269779752)

    def test_regular_update(self):
        bus = MagicMock()
        drone = Drone(bus=bus, config={})
        drone.on_desired_speed([1000, 0])  # fly forward for ever
        bus.publish.assert_called_with('desired_speed_3d', [[1.0, 0.0, 0.0], [0.0, 0.0, 0.0]])

        bus.reset_mock()
        drone.on_top_scan([10.1])  # order is unfortunately not defined -> extra trigger is on bottom scan
        drone.on_bottom_scan([0.1])
        bus.publish.assert_called_with('desired_speed_3d', [[1.0, 0.0, 2.5], [0.0, 0.0, 0.0]])


# vim: expandtab sw=4 ts=4
