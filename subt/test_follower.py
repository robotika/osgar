
import unittest
from unittest.mock import MagicMock, patch, call

from subt.follower import RadioFollower


class RadioFollowerTest(unittest.TestCase):

    def test_usage(self):
        bus = MagicMock()
        follower = RadioFollower(bus=bus, config={})
        follower.on_robot_name(b'A100L')
        self.assertIsNone(follower.robot_name_prefix)
        follower.on_robot_name(b'A100EXG')
        self.assertEqual(follower.robot_name_prefix, 'G')

# vim: expandtab sw=4 ts=4
