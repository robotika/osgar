import unittest
from unittest.mock import MagicMock
from datetime import timedelta

from osgar.bus import Bus, BusShutdownException
from osgar.node import Node


class NodeTest(unittest.TestCase):

    def test_shutdown_messages_keep_node_alive_until_close(self):
        bus = Bus(MagicMock(write=MagicMock(return_value=timedelta())))
        node = Node({}, bus.handle('controller'))
        bus.enable_shutdown('controller')
        with self.assertRaises(RuntimeError):
            node.bus.shutdown_ready()
        bus.send_shutdown('controller', {'phase': 'prepare'})
        self.assertEqual(node.update(), 'shutdown')
        self.assertTrue(node.bus.shutdown_ready_event.is_set())
        self.assertTrue(node.bus.is_alive())
        node.bus.shutdown_ready()  # repeated readiness is harmless
        bus.send_shutdown('controller', {'phase': 'close'})
        self.assertEqual(node.update(), 'shutdown')
        self.assertFalse(node.bus.is_alive())
        with self.assertRaises(BusShutdownException):
            node.update()

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
        with self.assertRaises(AssertionError):
            node.update()

    def test_update2(self):
        class VelNode(Node):
            def on_vel(self, data):
                self.vel = data
        empty_config = {}
        bus = Bus(logger=MagicMock())
        node = VelNode(config=empty_config, bus=bus.handle('mynode'))
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
