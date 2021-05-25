import unittest
from unittest.mock import MagicMock

from subt.artf_filter import ArtifactFilter
from subt.artifacts import GAS



class ArtifactFilterTest(unittest.TestCase):

    def test_usage(self):
        bus = MagicMock()
        filter = ArtifactFilter(bus=bus, config={})
        filter.on_robot_name(b'A100L')
        filter.on_localized_artf([GAS, [100.0, 0.0, 0.0]])
        bus.publish.assert_called_with('artf_xyz', [[GAS, [100000, 0, 0], 'A100L', None]])

    def test_maybe_remember_artifact(self):
        bus = MagicMock()
        filter = ArtifactFilter(bus=bus, config={})

        artf_data = ['TYPE_BACKPACK', -1614, 1886]
        artf_xyz = (0, 0, 0)
        self.assertTrue(filter.maybe_remember_artifact(artf_data, artf_xyz))

        # 2nd report should be ignored
        self.assertEqual(filter.maybe_remember_artifact(artf_data, artf_xyz), False)

    def test_staging_area(self):
        bus = MagicMock()
        filter = ArtifactFilter(bus=bus, config={})
        filter.on_robot_name(b'A100L')
        filter.on_localized_artf(['TYPE_DRILL', [-10.0, 0.0, 0.0]])
        bus.publish.assert_not_called()  # inside staging area

    def test_false_drill(self):
        # the breadcrumbs are sometimes wrongly classified as drill
        bus = MagicMock()
        filter = ArtifactFilter(bus=bus, config={})
        filter.on_robot_name(b'A100L')
        filter.on_breadcrumb([100.0, 20.0, -30.0])
        filter.on_localized_artf(['TYPE_DRILL', [100.0, 20.0, -30.0]])
        bus.publish.assert_not_called()  # inside staging area

        filter.on_localized_artf(['TYPE_DRILL', [105.0, 20.0, -30.0]])
        bus.publish.assert_called_with('artf_xyz', [['TYPE_DRILL', [105000, 20000, -30000], 'A100L', None]])

    def test_artf_confirmation(self):
        bus = MagicMock()
        filter = ArtifactFilter(bus=bus, config={'num_confirm': 1})
        self.assertFalse(filter.maybe_remember_artifact('TYPE_BACKPACK', [10, 20, 3]))
        self.assertTrue(filter.maybe_remember_artifact('TYPE_BACKPACK', [10.1, 20, 3]))  # confirmed
        self.assertFalse(filter.maybe_remember_artifact('TYPE_BACKPACK', [10.1, 20.1, 3]))  # already reported

# vim: expandtab sw=4 ts=4
