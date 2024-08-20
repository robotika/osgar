import unittest
from unittest.mock import MagicMock

from osgar.followme import FollowMe, EmergencyStopException
from osgar.bus import Bus


class FollowMeTest(unittest.TestCase):

    def test_usage(self):
        bus = Bus(MagicMock())
        app = FollowMe(config={}, bus=bus.handle('app'))
        tester = bus.handle('tester')
        tester.register('emergency_stop')
        bus.connect('tester.emergency_stop', 'app.emergency_stop')
        tester.publish('emergency_stop', True)

        app.raise_exception_on_stop = True
        
        with self.assertRaises(EmergencyStopException):
            app.followme()

    def test_pushme(self):
        bus = Bus(MagicMock())
        app = FollowMe(config={'action': 'push'}, bus=bus.handle('app'))
        scan = [10000] * 360
        app.on_scan(scan)
        ret = app.followme_step(scan, index=None)

# vim: expandtab sw=4 ts=4
