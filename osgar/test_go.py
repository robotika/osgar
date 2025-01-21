import unittest
from unittest.mock import MagicMock

from osgar.go import Go


class GoTest(unittest.TestCase):

    def test_usage(self):
        bus = MagicMock()
        app = Go(bus=bus, config={
            # required parameters
            'max_speed': 0.5,
            'dist': 1.0,
            'timeout': 10
        })
        app.on_pose2d([0, 0, 0])
#        bus.publish.assert_called_with('emergency_stop', True)
