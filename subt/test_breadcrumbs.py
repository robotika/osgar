import unittest
from unittest.mock import MagicMock

from subt.breadcrumbs import Breadcrumbs


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

# vim: expandtab sw=4 ts=4
