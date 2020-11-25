import unittest
from unittest.mock import MagicMock
import datetime

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
        tester.publish('origin', [1000, 9000])
        c.request_stop()
        c.join()
        self.assertEqual(tester.listen()[2], [[1.0, 0.0, 0.0], [0.0, 0.0, MAX_ANGULAR]])


# vim: expandtab sw=4 ts=4

