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

    def test_control(self):
        max_speed = 0.5
        bus = Bus(MagicMock())
        config = {
            'max_speed': max_speed,
            'path': [[0, 0], [3, 0], [3, 3]]
        }
        app = FollowPath(config=config, bus=bus.handle('app'))
        self.assertEqual(app.control([0, 0, 0]), (max_speed, 0))

        self.assertEqual(app.control([0, 0, 0.1]), (max_speed, -0.1))

    def test_follow_path(self):
        bus = Bus(MagicMock())
        config = {
            'path': [[0, 0], [3, 0]]
        }
        app = FollowPath(config=config, bus=bus.handle('app'))



# vim: expandtab sw=4 ts=4
