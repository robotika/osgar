
import unittest
from unittest.mock import MagicMock
from datetime import timedelta

from osgar.platforms.matty import Matty


class MattyTest(unittest.TestCase):

    def test_usage(self):
        bus = MagicMock()
        robot = Matty(bus=bus, config={})
        robot.on_esp_data(b'1234')
#        bus.publish.assert_called_with('emergency_stop', True)
