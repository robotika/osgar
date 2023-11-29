import unittest
from unittest.mock import MagicMock
from datetime import timedelta

from osgar.drivers.vanjee import VanJeeLidar

class VanJeeLidarTest(unittest.TestCase):

    def test_usage(self):
        config = {}
        handler = MagicMock()
        lidar = VanJeeLidar(config, bus=handler)

# vim: expandtab sw=4 ts=4
