import unittest
from unittest.mock import patch, MagicMock
import datetime

from osgar.drivers.realsense import RealSense
from osgar.bus import Bus


class RealSenseTest(unittest.TestCase):
    def test_spin_once(self):
        logger = MagicMock()
        logger.write = MagicMock(return_value=datetime.timedelta(microseconds=9721))
        bus = Bus(logger)
        with patch('osgar.drivers.realsense.rs') as mock:
            instance = mock.return_value
            c = RealSense(bus=bus.handle('rs'), config={})
            tester = bus.handle('tester')
            tester.register('tick')
            bus.connect('tester.tick', 'rs.trigger')
            c.start()
            tester.publish('tick', None)
            c.request_stop()
            c.join()

# vim: expandtab sw=4 ts=4
