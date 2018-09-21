import unittest
import time
from unittest.mock import MagicMock

from osgar.drivers.timer import Timer
from osgar.bus import BusHandler


class TimerTest(unittest.TestCase):
   
    def test_usage(self):
        config = {'sleep': 0.2}
        logger = MagicMock()
        bus = BusHandler(logger, out={'tick':[]}, name='timer')
        bus.publish = MagicMock()

        timer = Timer(config, bus=bus)
        timer.start()
        time.sleep(0.1)
        timer.request_stop()
        timer.join()
        bus.publish.assert_called_with('tick', 0)

    def test_fast_timer(self):
        config = {'sleep': 0.01}
        logger = MagicMock()
        bus = BusHandler(logger, out={'tick':[]}, name='timer')
        bus.publish = MagicMock()

        timer = Timer(config, bus=bus)
        timer.start()
        time.sleep(0.1)
        timer.request_stop()
        timer.join()

        # small tolerance for system performance
        self.assertIn(len(bus.publish.call_args_list), [9, 10, 11])

# vim: expandtab sw=4 ts=4
