import unittest
from unittest.mock import patch, MagicMock

from osgar.drivers.marina import Marina, word_arr
from osgar.bus import BusHandler


class MarinaTest(unittest.TestCase):

    def test_usage(self):
        logger = MagicMock()
        bus = BusHandler(logger, out={'cmd':[]})
        config = {}
        boat = Marina(config=config, bus=bus)
        bus.queue.put((1, 'move', [1000, 1000]))       
        boat.start()
        boat.request_stop()
        boat.join()

    def test_word_arr(self):
        self.assertEqual(word_arr(1), [1, 0])


# vim: expandtab sw=4 ts=4

