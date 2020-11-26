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

        bread.on_external([5, 0, 0])  # i.e. not within the 10m radius, but it is already placed
        self.assertEqual(bread.locations, [[0, 0, 0], [11, 0, 0], [5, 0, 0]])

    def test_limited_size(self):
        bus = MagicMock()
        bread = Breadcrumbs(bus=bus, config={'radius':10})
        self.assertEqual(bread.locations, [[0, 0, 0]])

        for x in range(15, 1000, 15):
            bread.on_pose3d([[x, 0, 0], quaternion.identity()])

        self.assertEqual(len(bread.locations), 1 + 6)

    def test_num_available(self):
        bus = MagicMock()
        bread = Breadcrumbs(bus=bus, config={'radius':10})
        self.assertEqual(bread.num_avail, 6)

        bread = Breadcrumbs(bus=bus, config={'radius':10, 'num': 12})
        self.assertEqual(bread.num_avail, 12)


# vim: expandtab sw=4 ts=4
