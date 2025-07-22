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
