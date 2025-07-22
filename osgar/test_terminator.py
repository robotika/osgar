import unittest
from unittest.mock import MagicMock

from osgar.terminator import Terminator
from osgar.followme import EmergencyStopException

class TerminatorTest(unittest.TestCase):

    def test_usage(self):
        bus = MagicMock()
        app = Terminator(bus=bus, config={})
        with self.assertRaises(EmergencyStopException):
            app.on_terminate_if_false(False)

    def test_change_required(self):
        bus = MagicMock()
        app = Terminator(bus=bus, config={
            "change_required": True
        })
        # Emergency STOP is pressed, but we require change -> ignore
        app.on_terminate_if_false(False)
        # now it was released so change is fulfilled
        app.on_terminate_if_false(True)
        # and now it is finally rising exception on STOP release
        with self.assertRaises(EmergencyStopException):
            app.on_terminate_if_false(False)

    def test_change_required_inv(self):
        bus = MagicMock()
        app = Terminator(bus=bus, config={
            "change_required": True
        })
        # Emergency STOP is pressed, but we require change -> ignore
        app.on_terminate_if_true(True)
        # now it was released so change is fulfilled
        app.on_terminate_if_true(False)
        # and now it is finally rising exception on STOP release
        with self.assertRaises(EmergencyStopException):
            app.on_terminate_if_true(True)
