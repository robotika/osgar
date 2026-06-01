import unittest
from unittest.mock import MagicMock

from osgar.platforms.tello import TelloDrone


class TelloDroneTest(unittest.TestCase):

    def test_usage(self):
        r = TelloDrone({}, MagicMock())

    def test_custom_tasks_parsing(self):
        bus = MagicMock()
        config = {
            "tasks": [
                [1.5, "takeoff"],
                [5.0, "land"]
            ]
        }
        drone = TelloDrone(config=config, bus=bus)
        self.assertEqual(drone.tasks, [
            [1.5, b'takeoff'],
            [5.0, b'land']
        ])

# vim: expandtab sw=4 ts=4
