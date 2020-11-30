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
        bus.connect('offseter.pose3d', 'tester.pose3d')
        c.start()
        tester.publish('origin', [b'A300W600R', -6.4, 5.0, 0.8, 0.0, 0.0, 1.0, 0])
        tester.publish('pose3d', [[0, 0, 0], [0, 0, 0, 1]])  # first pose is "consumed"
        tester.publish('pose3d', [[-2, -1, 0], [0, 0, 0, 1]])
        tester.publish('pose3d', [[-2, -3, 1], [0, 0, 0.7071068, 0.7071068]])  # 90 deg
        c.request_stop()
        c.join()
        self.assertIsNotNone(c.init_quat)
        xyz, ori = tester.listen()[2]
        self.assertEqual(xyz, [-6.4 + 2, 5.0 + 1, 0.8])
        self.assertEqual(ori, [0.0, 0.0, 1.0, 0])

        xyz, ori = tester.listen()[2]
        self.assertEqual(xyz, [-6.4 + 2, 5.0 + 3, 0.8 + 1])
        self.assertEqual(ori, [0, 0, 0.7071068, -0.7071068])  # 270 deg

# vim: expandtab sw=4 ts=4

