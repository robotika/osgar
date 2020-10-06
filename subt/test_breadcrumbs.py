import unittest
from unittest.mock import MagicMock

from subt.breadcrumbs import Breadcrumbs
from osgar.lib import quaternion

class BreadcrumbsTest(unittest.TestCase):

    def test_deploy(self):
        bus = bus=MagicMock()
        bread = Breadcrumbs(bus=bus, config={'step_sec':10})
        bread.on_sim_time_sec(3)
        bus.publish.assert_not_called()
        bread.on_sim_time_sec(14)
        bus.publish.assert_called()
        bus.publish.reset_mock()
        bread.on_sim_time_sec(15)
        bus.publish.assert_not_called()

    def test_breadcrumbs_dist(self):
        bus = bus=MagicMock()
        bread = Breadcrumbs(bus=bus, config={'radius':10})
        self.assertEqual(bread.locations, [[0, 0, 0]])

        bread.on_pose3d([[1, 0, 0], quaternion.identity()])
        self.assertEqual(bread.locations, [[0, 0, 0]])
        bus.publish.assert_not_called()

        bread.on_pose3d([[11, 0, 0], quaternion.identity()])
        self.assertEqual(bread.locations, [[0, 0, 0], [11, 0, 0]])
        bus.publish.assert_called()

    def test_external_breadcrumbs(self):
        bus = bus=MagicMock()
        bread = Breadcrumbs(bus=bus, config={'radius':10})
        self.assertEqual(bread.locations, [[0, 0, 0]])

        bus.publish.reset_mock()
        bread.on_external([11, 0, 0])
        self.assertEqual(bread.locations, [[0, 0, 0], [11, 0, 0]])

# vim: expandtab sw=4 ts=4
