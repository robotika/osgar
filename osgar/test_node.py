import unittest
from unittest.mock import MagicMock
from datetime import timedelta

from osgar.bus import Bus
from osgar.node import Node


class NodeTest(unittest.TestCase):

    def test_usage(self):
        empty_config = {}
        bus = Bus(logger=MagicMock())
        node = Node(config=empty_config, bus=bus.handle('mynode'))
        node.start()
        node.request_stop()
        node.join()

    def test_update(self):
        empty_config = {}
        bus = Bus(logger=MagicMock())
        node = Node(config=empty_config, bus=bus.handle('mynode'))
        tester = bus.handle('tester')
        tester.register('vel')
        bus.connect('tester.vel', 'mynode.vel')
        dt = tester.publish('vel', 3)
        node.update()
        self.assertEqual(node.time, dt)
        self.assertEqual(node.vel, 3)

        node2 = Node(config=empty_config, bus=bus.handle('mynode2'))
        self.assertNotIn('vel', dir(node2))

# vim: expandtab sw=4 ts=4
