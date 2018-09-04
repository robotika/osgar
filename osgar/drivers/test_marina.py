import unittest
from unittest.mock import patch, MagicMock

from osgar.drivers.marina import Marina
from osgar.bus import BusHandler


class LogI2CTest(unittest.TestCase):

    def test_usage(self):
        logger = MagicMock()
        bus = BusHandler(logger, out={'cmd':[]})
        config = {}
        boat = Marina(config=config, bus=bus)
        boat.start()
        boat.request_stop()
        boat.join()

# vim: expandtab sw=4 ts=4

