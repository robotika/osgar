import datetime
import unittest
from unittest.mock import MagicMock, patch, call

from osgar.bus import Bus
from subt.teambase import Teambase


class TeambaseTest(unittest.TestCase):

    def test_finish_time(self):
        tb = Teambase(bus=MagicMock(), config={'robot_name':'T42'})
        self.assertEqual(tb.finish_time, 42)

    def test_on_robot_xyz(self):
        tb = Teambase(bus=MagicMock(), config={'robot_name':'T300'})
        self.assertEqual(tb.robot_positions, {})
        data = ['A0F150L', [11896, 7018, -11886]]
        tb.on_robot_xyz(data)
        self.assertEqual(tb.robot_positions, {'A0F150L': [11896, 7018, -11886]})

# vim: expandtab sw=4 ts=4
