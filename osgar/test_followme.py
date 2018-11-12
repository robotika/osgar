import unittest
from unittest.mock import MagicMock
import math

from osgar.followme import FollowMe, EmergencyStopException
from osgar.bus import BusHandler


class FollowMeTest(unittest.TestCase):

    def test_usage(self):
        bus = BusHandler(name='app', logger=MagicMock)
        app = FollowMe(config={}, bus=bus)

        bus.queue.put((0, 'emergency_stop', True))
        app.raise_exception_on_stop = True
        
        with self.assertRaises(EmergencyStopException):
            app.followme()

# vim: expandtab sw=4 ts=4
