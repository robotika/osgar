import datetime
import math
import unittest
from unittest.mock import MagicMock

from osgar.bus import Bus
from subt.twistwrap import TwistWrap


class TwistWrapTest(unittest.TestCase):

    def test_usage(self):
        logger = MagicMock()
        bus = Bus(logger)
        logger.write = MagicMock(return_value=datetime.timedelta(microseconds=9721))
        c = TwistWrap(bus=bus.handle('twister'), config={})
        tester = bus.handle('tester')
        tester.register('desired_speed')
        bus.connect('tester.desired_speed', 'twister.desired_speed')
        bus.connect('twister.cmd_vel', 'tester.cmd_vel')
        c.start()
        tester.publish('desired_speed', [1000, 9000])
        c.request_stop()
        c.join()
        self.assertEqual(tester.listen()[2], [[1.0, 0.0, 0.0], [0.0, 0.0, math.radians(90)]])

# vim: expandtab sw=4 ts=4
