import unittest
from unittest.mock import MagicMock, call

from subt.marsupial import Marsupial
from osgar.lib import quaternion

class MarsupialTest(unittest.TestCase):

    def test_detach(self):
        bus = bus=MagicMock()
        robot = Marsupial(bus=bus, config={'release_at':10})
        robot.on_sim_time_sec(3)
        bus.publish.assert_not_called()
        robot.on_sim_time_sec(14)
        bus.publish.assert_called()
        bus.publish.assert_has_calls([call('detach', [])])
        bus.publish.reset_mock()
        robot.on_sim_time_sec(15)
        bus.publish.assert_not_called()

# vim: expandtab sw=4 ts=4
