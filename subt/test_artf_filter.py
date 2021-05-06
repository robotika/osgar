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


# vim: expandtab sw=4 ts=4
