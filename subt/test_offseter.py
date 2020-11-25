import unittest
from unittest.mock import MagicMock
import datetime
import math

from subt.offseter import Offseter
from osgar.bus import Bus


class OffseterTest(unittest.TestCase):

    def test_usage(self):
        logger = MagicMock()
        bus = Bus(logger)
        logger.write = MagicMock(return_value=datetime.timedelta(microseconds=9721))
        c = Offseter(bus=bus.handle('offseter'), config={})
        tester = bus.handle('tester')
        tester.register('origin', 'pose3d')
        bus.connect('tester.origin', 'offseter.origin')
        bus.connect('tester.pose3d', 'offseter.pose3d')
        c.start()
        tester.publish('origin', [b'A300W600R', -6.409662, 5.000274, 0.805016, -0.005923, -1.8e-05, 0.999982, -1e-05])
        tester.publish('pose3d', [[2.2540965890777053e-05, -1.7848595179941423e-05, -5.095526932796975e-12], [0, 0, 0, 1]])
        c.request_stop()
        c.join()
        self.assertEqual(c.yaw_offset, math.pi)

# vim: expandtab sw=4 ts=4

