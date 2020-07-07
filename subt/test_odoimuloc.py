
import unittest
from unittest.mock import MagicMock
from datetime import timedelta

from osgar.bus import Bus
from osgar.lib import quaternion
from subt.odoimuloc import Localization

class Test(unittest.TestCase):

    def test_zero(self):
        bus = Bus(MagicMock(write=MagicMock(return_value=timedelta())))
        loc = Localization({}, bus.handle('loc'))
        tester = bus.handle('tester')
        tester.register('origin', 'orientation', 'odom')
        bus.connect('loc.pose3d', 'tester.pose3d')
        bus.connect('tester.origin', 'loc.origin')
        bus.connect('tester.orientation', 'loc.orientation')
        bus.connect('tester.odom', 'loc.odom')
        loc.start()
        origin = [0, 0, 0]
        tester.publish('origin', ['name'] + origin + quaternion.identity())
        tester.publish('orientation', quaternion.identity())
        tester.publish('odom', origin)
        dt, channel, pose3d = tester.listen()
        self.assertEqual(channel, 'pose3d')
        self.assertEqual(pose3d, [origin, quaternion.identity()])
        loc.request_stop()
        loc.join()


