import unittest
from unittest.mock import MagicMock

from moon.monitors import (LidarCollisionException, LidarCollisionMonitor,
                           VirtualBumperMonitor)


class MonitorTester:
    def __init__(self):
        self.monitors = []

    def register(self, callback):
        self.monitors.append(callback)
        return callback

    def unregister(self, callback):
        assert callback in self.monitors
        self.monitors.remove(callback)

    def update(self, channel):
        data = getattr(self, channel)
        for m in self.monitors:
            m(self, channel, data)

    def publish(self, channel, data):
        for m in self.monitors:
            m(self, channel, data)


class MoonMonitorsTest(unittest.TestCase):

    def test_lidar_monitor(self):
        robot = MonitorTester()
        with LidarCollisionMonitor(robot):
            robot.scan = [3000]*270
            robot.update('scan')

    def test_virtual_bumper_monitor(self):
        robot = MonitorTester()
        with VirtualBumperMonitor(robot, virtual_bumper=MagicMock()):
            robot.publish('desired_speed', [0, 0])

# vim: expandtab sw=4 ts=4

