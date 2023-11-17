
import unittest
from unittest.mock import MagicMock
from datetime import timedelta

from osgar.platforms.yuhesen import FR07


class FR07Test(unittest.TestCase):

    def test_usage(self):
        bus = MagicMock()
        robot = FR07(bus=bus, config={})
        robot.on_can([0x18c4eaef, bytes.fromhex('3200000000704002'), 1])
        bus.publish.assert_called_with('emergency_stop', True)
