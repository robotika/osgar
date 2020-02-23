import unittest
import math
from unittest import mock
from datetime import timedelta
from subt import monitors

class Robot:

    def __init__(self):
        self.callbacks = []
        self.time = timedelta()
        self.pitch = 0
        self.roll = 0

    def update(self, dt=timedelta(milliseconds=101)):
        self.time += dt
        self.pitch += 0.1
        self.roll += 0.15
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
        timeout = monitors.TimeoutMonitor(robot, timedelta(seconds=1))
        with timeout:
            for _ in range(100):
                robot.update()
            else:
                self.assertTrue(False)
        self.assertGreater(robot.time, timedelta(seconds=1))
        self.assertTrue(timeout)

    def test_stacked_timeout(self):
        robot = Robot()
        timeout_outside = monitors.TimeoutMonitor(robot, timedelta(seconds=1))
        timeout_inside = monitors.TimeoutMonitor(robot, timedelta(seconds=0.5))
        with timeout_outside:
            with timeout_inside:
                for _ in range(100):
                    robot.update()
            self.assertGreater(robot.time, timedelta(seconds=0.5))
            self.assertLess(robot.time, timedelta(seconds=1))
            self.assertTrue(timeout_inside)
            self.assertFalse(timeout_outside)
            for _ in range(100):
                robot.update()
        self.assertGreater(robot.time, timedelta(seconds=1))
        self.assertTrue(timeout_outside)


class PitchRollTest(unittest.TestCase):

    def test_pitch(self):
        robot = Robot()
        pitch_limit = monitors.PitchMonitor(robot, math.radians(80))
        with pitch_limit:
            for _ in range(100):
                robot.update()
            else:
                self.assertTrue(False)
        self.assertTrue(pitch_limit)

    def test_roll(self):
        robot = Robot()
        roll_limit = monitors.RollMonitor(robot, math.radians(80))
        with roll_limit:
            for _ in range(100):
                robot.update()
            else:
                self.assertTrue(False)
        self.assertTrue(roll_limit)

    def test_stacked_roll(self):
        robot = Robot()
        max_roll_limit = monitors.RollMonitor(robot, math.radians(80))
        with max_roll_limit:
            mid_roll_limit = monitors.RollMonitor(robot, math.radians(20))
            with mid_roll_limit:
                for _ in range(100):
                    robot.update()
                else:
                    self.assertTrue(False)
            self.assertTrue(mid_roll_limit)
            self.assertFalse(max_roll_limit)
            for _ in range(100):
                robot.update()
            else:
                self.assertTrue(False)
            self.assertTrue(mid_roll_limit)
            self.assertTrue(max_roll_limit)

    def test_roll_logging(self):
        with mock.patch("subt.monitors.g_logger") as logger:
            robot = Robot()
            roll = monitors.RollMonitor(robot, math.radians(20))
            with roll:
                robot.roll = 2
                robot.update()
            self.assertTrue(logger.info.called)
