import datetime
import unittest
from unittest.mock import MagicMock, patch, call

from osgar.bus import Bus
from subt.teambase import Teambase


class TeambaseTest(unittest.TestCase):

    def test_usage(self):
        logger = MagicMock()
        bus = Bus(logger)
        logger.write = MagicMock(return_value=datetime.timedelta(microseconds=9721))
        c = Teambase(bus=bus.handle('teambase'), config={})
        tester = bus.handle('tester')
        tester.register('sim_time_sec')
        bus.connect('tester.sim_time_sec', 'teambase.sim_time_sec')
        bus.connect('teambase.broadcast', 'tester.broadcast')
        c.start()
        tester.publish('sim_time_sec', 13)
        c.request_stop()
        c.join()
        self.assertEqual(tester.listen()[2], b'13')

    def test_finish_time(self):
        tb = Teambase(bus=MagicMock(), config={'robot_name':'T42'})
        self.assertEqual(tb.finish_time, 42)

# vim: expandtab sw=4 ts=4
