import unittest
from unittest.mock import MagicMock
from datetime import timedelta

from osgar.drivers.spider import Spider, CAN_triplet, sint8_diff
from osgar.bus import Bus


class SpiderTest(unittest.TestCase):

    def test_publish_status(self):
        logger=MagicMock()
        logger.write = MagicMock(return_value=timedelta(seconds=135))
        bus = Bus(logger=logger)
        tester = bus.handle('tester')
        tester.register('can')
        spider = Spider(config={}, bus=bus.handle('spider'))
        bus.connect('tester.can', 'spider.can')
        bus.connect('spider.status', 'tester.status')

        tester.publish('can', CAN_triplet(0x200, [0, 0x80]))
        spider.start()
        dt, stream, data = tester.listen()
        spider.request_stop()
        spider.join()
        self.assertEqual(dt, timedelta(seconds=135))
        self.assertEqual(stream, 'status')
        self.assertEqual(data, ([0x8000, None]))

    def test_8bit_diff(self):
        self.assertEqual(sint8_diff(0, 256), 0)
        self.assertEqual(sint8_diff(0, 1), -1)
        self.assertEqual(sint8_diff(255, 0), -1)


# vim: expandtab sw=4 ts=4
