import unittest
import math

from osgar.drivers.simulator import SpiderSimulator
from osgar.drivers.bus import BusHandler


class SpiderSimulatorTest(unittest.TestCase):

    def test_usage(self):
        config = {'position': [51748232, 180462051]}
        bus = None  # BusHandler(logger)
        spider = SpiderSimulator(config, bus)

        spider.desired_speed = 0
        spider.desired_wheel_angle = 0
        spider.step(0.1)
        self.assertEqual(spider.rel_pose, (0, 0, 0))

        spider.desired_speed = 10
        spider.desired_wheel_angle = 0
        spider.step(2.0)
        self.assertGreater(spider.rel_pose[0], 0.0)

        spider.desired_speed = 1
        spider.desired_wheel_angle = math.radians(45)
        spider.status = 0x3
        spider.step(1.0)
        self.assertGreater(spider.wheel_angle, 0.0)
        self.assertEqual(spider.status, 0x8003)  # alive bit
        spider.step(1.0)
        self.assertEqual(spider.status, 0x3)

    def test_get_position(self):
        config = {'position': [51748232, 180462051]}
        spider = SpiderSimulator(config=config, bus=None)
        p = spider.get_position()
        self.assertEqual(p, [51748232, 180462051])

        spider.rel_pose = (100, 0, 0)  # 100m to East
        p = spider.get_position()
        self.assertEqual(p, [51748232 + 3344, 180462051])

        spider.rel_pose = (0, -1, 0)  # 1m to South
        p = spider.get_position()
        self.assertEqual(p, [51748232, 180462051 - 33])

        spider.rel_pose = (100, 0, 0)  # 100m to East on eqator
        spider.ref_position = [0, 180462051]
        p = spider.get_position()
        self.assertEqual(p, [3240, 180462051])

    def test_get_wheels_angles_raw(self):
        config = {'position': [51748232, 180462051]}
        spider = SpiderSimulator(config=config, bus=None)
        angles = spider.get_wheels_angles_raw()
        self.assertEqual(angles, [0, 0, 0, 0])

        spider.wheel_angle = math.radians(270)
        angles = spider.get_wheels_angles_raw()
        self.assertEqual(angles, [128, 128, 128, 128])

# vim: expandtab sw=4 ts=4
