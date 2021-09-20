import unittest
from unittest.mock import MagicMock, call
from pathlib import Path

from subt.artf_reporter import ArtifactReporter, artf_sort_fcn


class ArtifactReporterTest(unittest.TestCase):

    def test_artf_repeat(self):
        # default behavior
        config = {}
        bus = MagicMock()
        reporter = ArtifactReporter(config, bus)
        bus.listen = MagicMock(return_value=(1, 'artf_xyz', [['TYPE_BACKPACK', [100, 200, -3], "robot-name", None]]))
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
        bus.listen = MagicMock(return_value=(1, 'artf_xyz', [['TYPE_BACKPACK', [100, 200, -3], "name", None]]))
        reporter.update()
        bus.publish.assert_called()
        bus.reset_mock()
        for sim_time_sec in range(20):
            bus.listen = MagicMock(return_value=(1, 'sim_time_sec', sim_time_sec))
            reporter.update()
        # well, I am not sure how to ensure only 2 calls :(
        bus.publish.assert_has_calls([call('artf_cmd', b'artf TYPE_BACKPACK 0.10 0.20 -0.00\n'),
                                      call('artf_lora', ['TYPE_BACKPACK', 100, 200, -3]),
                                      call('artf_all', [['TYPE_BACKPACK', [100, 200, -3], "name", None]]),]*2)

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
        bus.publish.assert_has_calls([call('artf_cmd', b'artf TYPE_BACKPACK 0.10 0.20 -0.00\n')])
        # this 2nd call is postponed - call('artf_cmd', b'artf TYPE_ROPE 0.01 0.02 0.03\n')])

        # keep list unique
        bus.listen = MagicMock(return_value=(1, 'artf_xyz', [['TYPE_BACKPACK', [100, 200, -3], "RobotX", None]]))
        reporter.update()
        bus.reset_mock()
        sim_time_sec = 0
        bus.listen = MagicMock(return_value=(1, 'sim_time_sec', sim_time_sec))
        reporter.update()
        # the same issue, that I do not want subset, but 1:1 correspondence - TODO more strict assert
        bus.publish.assert_has_calls([call('artf_cmd', b'artf TYPE_BACKPACK 0.10 0.20 -0.00\n')])
        # this 2nd call is postponed - call('artf_cmd', b'artf TYPE_ROPE 0.01 0.02 0.03\n')])

    def test_on_base_station(self):
        data = {'report_id': 1, 'artifact_type': 0, 'artifact_position': [69.84, -4.86, 0.4],
                'report_status': 'scored', 'score_change': 1}
        bus = MagicMock()
        reporter = ArtifactReporter(config={}, bus=bus)
        reporter.on_base_station(data)
        self.assertEqual(reporter.artf_xyz_accumulated, [])
        bus.publish.assert_not_called()  # nothing to report/share
        reporter.artf_xyz_accumulated = [['TYPE_BACKPACK', [69845, -4860, 396], 'A150L', None]]
        reporter.on_base_station(data)
        self.assertEqual(reporter.artf_xyz_accumulated, [['TYPE_BACKPACK', [69845, -4860, 396], 'A150L', True]])
        bus.publish.assert_called()  # make sure that new confirmed artifact is propagated

        bus.reset_mock()
        reporter.on_base_station(data)
        self.assertEqual(reporter.artf_xyz_accumulated, [['TYPE_BACKPACK', [69845, -4860, 396], 'A150L', True]])
        bus.publish.assert_not_called()  # the information did not change - no hurry to spread it

        data['score_change'] = 0  # wrongly reported artifact
        reporter.on_base_station(data)
        self.assertEqual(reporter.artf_xyz_accumulated, [['TYPE_BACKPACK', [69845, -4860, 396], 'A150L', False]])

    def test_on_artf_xyz(self):
        # update from detector or from radio
        bus = MagicMock()
        reporter = ArtifactReporter(config={}, bus=bus)
        reporter.artf_xyz_accumulated = [['TYPE_BACKPACK', [69758, -5143, 533], 'A150L', True], ['TYPE_BACKPACK', [70044, -5289, 438], 'B10W150L', False]]
        reporter.on_artf_xyz([['TYPE_BACKPACK', [69758, -5143, 533], 'A150L', None]])
        self.assertEqual(reporter.artf_xyz_accumulated, [['TYPE_BACKPACK', [69758, -5143, 533], 'A150L', True],
                                         ['TYPE_BACKPACK', [70044, -5289, 438], 'B10W150L', False]])

    def test_duplicity_reports(self):
        bus = MagicMock()
        reporter = ArtifactReporter(config={}, bus=bus)
        self.assertEqual(len(reporter.artf_xyz_accumulated), 0)
        reporter.on_artf_xyz([['TYPE_BACKPACK', [69758, -5143, 533], 'A150L', True]])
        reporter.on_artf_xyz([['TYPE_BACKPACK', [69758 + 1000, -5143, 533], 'A150L', None]])  # 1m offset
        self.assertEqual(len(reporter.artf_xyz_accumulated), 1)
        bus.publish.assert_not_called()

        # backpack is confirmed and phone is tested as different artifact
        reporter.on_artf_xyz([['TYPE_PHONE', [69758 + 2000, -5143, 533], 'A150L', None]])  # 2m offset
        self.assertEqual(len(reporter.artf_xyz_accumulated), 2)

    def test_select_artf_for_report(self):
        bus = MagicMock()
        reporter = ArtifactReporter(config={}, bus=bus)

        to_report = reporter.select_artf_for_report([
                   ['TYPE_DRILL', [343506, 22288, -14938], 'B300W900ELXA', None],
                   ['TYPE_DRILL', [343174, 22338, -14727], 'A900L', None],
                   ['TYPE_RESCUE_RANDY', [252978, 106894, -18309], 'A900L', None],
                   ['TYPE_RESCUE_RANDY', [252930, 106716, -19523], 'B300W900ELXA', None]])

        # pick only one sample (minimum, but avoid drill)
        self.assertEqual(to_report, [
            ['TYPE_RESCUE_RANDY', [252930, 106716, -19523], 'B300W900ELXA', None]
        ])

        # now let's have already positive report from B-drone
        to_report = reporter.select_artf_for_report([
                   ['TYPE_DRILL', [343506, 22288, -14938], 'B300W900ELXA', True],
                   ['TYPE_DRILL', [343174, 22338, -14727], 'A900L', None]])

        # keep only successful report
        self.assertEqual(to_report, [])

        # now let's have negative result of different artifact type
        to_report = reporter.select_artf_for_report([
                   ['TYPE_DRILL', [343506, 22288, -14938], 'B300W900ELXA', None],
                   ['TYPE_BACKPACK', [343174, 22338, -14727], 'A900L', False]])

        # keep only promising report
        self.assertEqual(to_report, [
            ['TYPE_DRILL', [343506, 22288, -14938], 'B300W900ELXA', None]
        ])

        # the same type of artifact but already received False for nearby location
        to_report = reporter.select_artf_for_report([
                   ['TYPE_BACKPACK', [343506, 22288, -14938], 'B300W900ELXA', None],
                   ['TYPE_BACKPACK', [343174, 22338, -14727], 'A900L', False]])

        # the report already failed, no need to report it again
        self.assertEqual(to_report, [])

        # two different reports from the same robot
        to_report = reporter.select_artf_for_report([
                   ['TYPE_ROPE', [343506, 22288, -14938], 'A900L', None],
                   ['TYPE_BACKPACK', [343174, 22338, -14727], 'A900L', None]])

        # keep only one of the options, verify and then try the second
        self.assertEqual(to_report, [
            ['TYPE_BACKPACK', [343174, 22338, -14727], 'A900L', None]
        ])

        # True artefact masks ALL other reports
        to_report = reporter.select_artf_for_report([
                   ['TYPE_ROPE', [343506, 22288, -14938], 'A900L', True],
                   ['TYPE_BACKPACK', [343174, 22338, -14727], 'A900L', None]])

        self.assertEqual(to_report, [])


    def test_artf_select_integration(self):
        bus = MagicMock()
        reporter = ArtifactReporter(config={}, bus=bus)

        reporter.on_artf_xyz([
                   ['TYPE_DRILL', [343506, 22288, -14938], 'B300W900ELXA', None],
                   ['TYPE_DRILL', [343174, 22338, -14727], 'A900L', None],
                   ['TYPE_RESCUE_RANDY', [252978, 106894, -18309], 'A900L', None],
                   ['TYPE_RESCUE_RANDY', [252930, 106716, -19523], 'B300W900ELXA', None]])

        bus.publish.assert_has_calls([
            call('artf_cmd', b'artf TYPE_RESCUE_RANDY 252.93 106.72 -19.52\n')])

    def test_grouping_order(self):
        # the order does matter in order to identically report artifacts
        bus = MagicMock()
        reporter = ArtifactReporter(config={}, bus=bus)

        artf_xyz = [['TYPE_ROPE', [343506, 22288, -14938], 'A900L', None],
                    ['TYPE_BACKPACK', [343174, 22338, -14727], 'A900L', None]]

        # two different reports from the same robot
        to_report = reporter.select_artf_for_report(artf_xyz)

        self.assertEqual(to_report, [
            ['TYPE_BACKPACK', [343174, 22338, -14727], 'A900L', None]
        ])

        artf_xyz.reverse()

        to_report = reporter.select_artf_for_report(artf_xyz)

        self.assertEqual(to_report, [
            ['TYPE_BACKPACK', [343174, 22338, -14727], 'A900L', None]
        ])

    def test_artifact_types_order(self):
        # the preferred order is (JL) backpack < extinguisher < drill, and cube < phone
        # where to original alphabetical sorting is issue only with extinguisher
        bus = MagicMock()
        reporter = ArtifactReporter(config={}, bus=bus)

        artf_xyz = [['TYPE_EXTINGUISHER', [343506, 22288, -14938], 'A900L', None],
                    ['TYPE_DRILL', [343174, 22338, -14727], 'A900L', None]]

        # two different reports from the same robot
        to_report = reporter.select_artf_for_report(artf_xyz)

        self.assertEqual(to_report, [
            ['TYPE_EXTINGUISHER', [343506, 22288, -14938], 'A900L', None]
        ])

    def test_artf_sort_fcn(self):
        artf_xyz = [['TYPE_DRILL', [343174, 22338, -14727], 'A900L', None],
                    ['TYPE_EXTINGUISHER', [343506, 22288, -14938], 'A900L', None]]

        # Drill has the lowest priority
        self.assertEqual(min(artf_xyz, key=artf_sort_fcn),
                         ['TYPE_EXTINGUISHER', [343506, 22288, -14938], 'A900L', None])

        artf_xyz = [['TYPE_BACKPACK', [-10000, 22338, -14727], 'B10W900R', None],
                    ['TYPE_BACKPACK', [343506, 22288, -14938], 'A900L', None]]

        # take coordinates into account
        self.assertEqual(min(artf_xyz, key=artf_sort_fcn),
                         ['TYPE_BACKPACK', [-10000, 22338, -14727], 'B10W900R', None])

        artf_xyz.reverse()
        # the results should be still the same
        self.assertEqual(min(artf_xyz, key=artf_sort_fcn),
                         ['TYPE_BACKPACK', [-10000, 22338, -14727], 'B10W900R', None])


# vim: expandtab sw=4 ts=4

