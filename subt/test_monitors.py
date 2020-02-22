import unittest
from datetime import timedelta
from random import Random
from subt import monitors

class Robot:
    def __init__(self):
        self.callbacks = []
        self.time = timedelta()
        self.random = Random(0)

    def update(self, dt=timedelta(milliseconds=101)):
        self.time += dt
        for f in self.callbacks:
            f()

    def register(self, callback):
        self.callbacks.append(callback)
        return callback

    def unregister(self, handle):
        assert handle in self.callbacks
        self.callbacks.remove(handle)


class TimeoutTest(unittest.TestCase):
    def test_timeout(self):
        robot = Robot()
        timeout = monitors.Timeout(robot, timedelta(seconds=1))
        with timeout:
            while True:
                robot.update()
        self.assertGreater(robot.time, timedelta(seconds=1))
        self.assertTrue(timeout)

    def test_stacked_timeout(self):
        robot = Robot()
        timeout_outside = monitors.Timeout(robot, timedelta(seconds=1))
        timeout_inside = monitors.Timeout(robot, timedelta(seconds=0.5))
        with timeout_outside:
            with timeout_inside:
                while True:
                    robot.update()
            self.assertGreater(robot.time, timedelta(seconds=0.5))
            self.assertLess(robot.time, timedelta(seconds=1))
            self.assertTrue(timeout_inside)
            self.assertFalse(timeout_outside)
            while True:
                robot.update()
        self.assertGreater(robot.time, timedelta(seconds=1))
        self.assertTrue(timeout_outside)

