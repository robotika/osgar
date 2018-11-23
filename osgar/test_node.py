import unittest
from unittest.mock import MagicMock
from datetime import timedelta

from osgar.bus import BusHandler
from osgar.node import Node


class NodeTest(unittest.TestCase):

    def test_usage(self):
        empty_config = {}
        bus = BusHandler(name='mynode', logger=MagicMock)
        node = Node(config=empty_config, bus=bus)
        node.start()
        node.request_stop()
        node.join()

    def test_update(self):
        empty_config = {}
        bus = BusHandler(name='mynode', logger=MagicMock)
        node = Node(config=empty_config, bus=bus)
        bus.queue.put((timedelta(seconds=1), 'vel', 3))
        node.update()
        self.assertEqual(node.time, timedelta(seconds=1))
        self.assertEqual(node.vel, 3)

        bus2 = BusHandler(name='mynode2', logger=MagicMock)
        node = Node(config=empty_config, bus=bus2)
        self.assertNotIn('vel', dir(node))

# vim: expandtab sw=4 ts=4
