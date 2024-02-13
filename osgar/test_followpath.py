import unittest
from unittest.mock import MagicMock

from osgar.followpath import FollowPath, EmergencyStopException
from osgar.bus import Bus


class FollowPathTest(unittest.TestCase):

    def test_usage(self):
        bus = Bus(MagicMock())
        app = FollowPath(config={}, bus=bus.handle('app'))
        tester = bus.handle('tester')
        tester.register('emergency_stop')
        bus.connect('tester.emergency_stop', 'app.emergency_stop')
        tester.publish('emergency_stop', True)

        app.raise_exception_on_stop = True
        
        with self.assertRaises(EmergencyStopException):
            app.run()

# vim: expandtab sw=4 ts=4
