import unittest
from unittest.mock import MagicMock, call
from pathlib import Path

from subt.artf_reporter import ArtifactReporter


class ArtifactReporterTest(unittest.TestCase):

    def test_artf_repeat(self):
        # default behavior
        config = {}
        bus = MagicMock()
        reporter = ArtifactReporter(config, bus)
        bus.listen = MagicMock(return_value=(1, 'artf_xyz', [['BACKPACK', [100, 200, -3], "robot-name", None]]))
        reporter.update()
        bus.publish.assert_called()
        bus.reset_mock()
        for sim_time_sec in range(20):
            bus.listen = MagicMock(return_value=(1, 'sim_time_sec', sim_time_sec))
            reporter.update()
        bus.publish.assert_not_called()

        # now with repetitions
        config = {
            'repeat_report_sec': 10
        }
        bus = MagicMock()
        reporter = ArtifactReporter(config, bus)
        bus.listen = MagicMock(return_value=(1, 'artf_xyz', [['BACKPACK', [100, 200, -3], "name", None]]))
        reporter.update()
        bus.publish.assert_called()
        bus.reset_mock()
        for sim_time_sec in range(20):
            bus.listen = MagicMock(return_value=(1, 'sim_time_sec', sim_time_sec))
            reporter.update()
        # well, I am not sure how to ensure only 2 calls :(
        bus.publish.assert_has_calls([call('artf_cmd', b'artf BACKPACK 0.10 0.20 -0.00\n'),
                                      call('artf_all', [['BACKPACK', [100, 200, -3], "name", None]]),]*2)

    def test_artf_empty_repeat(self):
        config = {
            'repeat_report_sec': 10
        }
        bus = MagicMock()
        reporter = ArtifactReporter(config, bus)
        sim_time_sec = 0
        bus.listen = MagicMock(return_value=(1, 'sim_time_sec', sim_time_sec))
        reporter.update()
        bus.publish.assert_not_called()

    def test_artf_report_aggregation(self):
        config = {
            'repeat_report_sec': 10
        }
        bus = MagicMock()
        reporter = ArtifactReporter(config, bus)
        bus.listen = MagicMock(return_value=(1, 'artf_xyz', [['TYPE_BACKPACK', [100, 200, -3], "RoboX", None]]))
        reporter.update()
        bus.listen = MagicMock(return_value=(1, 'artf_xyz', [['TYPE_ROPE', [10, 20, 30], "RobotX", None]]))
        reporter.update()
        bus.reset_mock()
        sim_time_sec = 0
        bus.listen = MagicMock(return_value=(1, 'sim_time_sec', sim_time_sec))
        reporter.update()
        bus.publish.assert_has_calls([call('artf_cmd', b'artf TYPE_BACKPACK 0.10 0.20 -0.00\n'),
                                      call('artf_cmd', b'artf TYPE_ROPE 0.01 0.02 0.03\n')])

        # keep list unique
        bus.listen = MagicMock(return_value=(1, 'artf_xyz', [['TYPE_BACKPACK', 100, 200, -3]]))
        reporter.update()
        bus.reset_mock()
        sim_time_sec = 0
        bus.listen = MagicMock(return_value=(1, 'sim_time_sec', sim_time_sec))
        reporter.update()
        # the same issue, that I do not want subset, but 1:1 correspondence - TODO more strict assert
        bus.publish.assert_has_calls([call('artf_cmd', b'artf TYPE_BACKPACK 0.10 0.20 -0.00\n'),
                                      call('artf_cmd', b'artf TYPE_ROPE 0.01 0.02 0.03\n')])

# vim: expandtab sw=4 ts=4

