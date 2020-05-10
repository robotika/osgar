import unittest

from moon.monitors import LidarCollisionException, LidarCollisionMonitor

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
        for m in self.monitors:
            m(self, channel)


class MoonMonitorsTest(unittest.TestCase):

    def test_lidar_monitor(self):
        robot = MonitorTester()
        with LidarCollisionMonitor(robot):
            robot.scan = [3000]*270
            robot.update('scan')

# vim: expandtab sw=4 ts=4

