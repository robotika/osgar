
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

        follower.on_robot_xyz(['G10W900L', [52, [-3.499999464866466, 3.9999992320291566, 0.11749785807827295]]])

        bus.reset_mock()
        follower.on_sim_time_sec(13)
        bus.publish.assert_called()

    def test_get_leader_robot_name(self):
        bus = MagicMock()
        follower = RadioFollower(bus=bus, config={})
        follower.robot_name_prefix = None
        self.assertIsNone(follower.get_leader_robot_name())

        follower.robot_name_prefix = 'A'
        self.assertIsNone(follower.get_leader_robot_name())

        follower.robot_name_prefix = 'A'
        follower.robot_names = ['A100L', 'B10W300R']
        self.assertEqual(follower.get_leader_robot_name(), 'A100L')

    def test_robot_names_collection(self):
        bus = MagicMock()
        follower = RadioFollower(bus=bus, config={})
        self.assertEqual(follower.robot_names, [])

        follower.on_robot_xyz(['B900R', [52, [-3.499999938402841, 1.999999616027235, 0.11749921263375614]]])
        self.assertEqual(follower.robot_names, ['B900R'])

        follower.on_robot_xyz(['A900L', [52, [-3.499999464866466, 3.9999992320291566, 0.11749785807827295]]])
        self.assertEqual(follower.robot_names, ['B900R', 'A900L'])

        follower.on_robot_xyz(['B900R', [53, [-3.799999938402841, 1.999999616027235, 0.11749921263375614]]])
        self.assertEqual(follower.robot_names, ['B900R', 'A900L'])

# vim: expandtab sw=4 ts=4
