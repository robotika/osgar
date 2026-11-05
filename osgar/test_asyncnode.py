import asyncio
import time
import unittest
from datetime import timedelta
from unittest.mock import MagicMock

from osgar.bus import Bus
from osgar.asyncnode import AsyncNode


def create_mock_logger():
    logger = MagicMock()
    logger.write = MagicMock(return_value=timedelta(seconds=1))
    return logger


class TestAsyncNode(unittest.TestCase):

    def test_stop_drains_queue_and_awaits_cleanup(self):
        class Transport(AsyncNode):
            def __init__(self, config, bus):
                super().__init__(config, bus)
                self.sent = []
                self.cleaned = False

            async def on_raw(self, data):
                await asyncio.sleep(0.01)
                self.sent.append(data)

            async def main(self):
                try:
                    await self.listen_loop()
                finally:
                    await asyncio.sleep(0.01)
                    self.cleaned = True

        bus = Bus(create_mock_logger())
        node = Transport({}, bus.handle('transport'))
        sender = bus.handle('sender')
        sender.register('raw')
        bus.connect('sender.raw', 'transport.raw')
        sender.publish('raw', b'STOP')
        node.request_stop()  # Queue the sentinel before the event loop starts.
        node.start()
        try:
            node.join(2)
            self.assertFalse(node.is_alive())
            self.assertEqual(node.sent, [b'STOP'])
            self.assertTrue(node.cleaned)
        finally:
            node.force_stop()
            node.join(2)

    def test_force_stop_before_start(self):
        class SleepingNode(AsyncNode):
            async def main(self):
                await asyncio.Event().wait()
        node = SleepingNode({}, Bus(create_mock_logger()).handle('node'))
        node.force_stop()
        node.start()
        node.join(2)
        self.assertFalse(node.is_alive())

    def test_usage(self):
        empty_config = {}
        bus = Bus(logger=create_mock_logger())
        node = AsyncNode(config=empty_config, bus=bus.handle('mynode'))
        node.start()
        node.request_stop()
        node.join(timeout=1.0)
        self.assertFalse(node.is_alive())

    def test_update_async(self):
        class AsyncReceiverNode(AsyncNode):
            def __init__(self, config, bus):
                super().__init__(config, bus)
                self.received = None

            async def on_data(self, data):
                await asyncio.sleep(0.01)
                self.received = data

        bus = Bus(logger=create_mock_logger())
        node = AsyncReceiverNode(config={}, bus=bus.handle('receiver'))
        tester = bus.handle('tester')
        tester.register('data')
        bus.connect('tester.data', 'receiver.data')

        node.start()
        tester.publish('data', 42)
        for _ in range(50):
            if node.received == 42:
                break
            time.sleep(0.01)
        self.assertEqual(node.received, 42)
        node.request_stop()
        node.join(timeout=1.0)
        self.assertFalse(node.is_alive())

    def test_sync_handler_in_async_node(self):
        class SyncReceiverNode(AsyncNode):
            def __init__(self, config, bus):
                super().__init__(config, bus)
                self.received = None

            def on_data(self, data):
                self.received = data

        bus = Bus(logger=create_mock_logger())
        node = SyncReceiverNode(config={}, bus=bus.handle('receiver'))
        tester = bus.handle('tester')
        tester.register('data')
        bus.connect('tester.data', 'receiver.data')

        node.start()
        tester.publish('data', 'hello')
        for _ in range(50):
            if node.received == 'hello':
                break
            time.sleep(0.01)
        self.assertEqual(node.received, 'hello')
        node.request_stop()
        node.join(timeout=1.0)
        self.assertFalse(node.is_alive())

    def test_sync_update_compat(self):
        class MyNode(AsyncNode):
            def __init__(self, config, bus):
                super().__init__(config, bus)
                self.received = None

            async def on_val(self, data):
                self.received = data * 2

        bus = Bus(logger=create_mock_logger())
        node = MyNode(config={}, bus=bus.handle('mynode'))
        tester = bus.handle('tester')
        tester.register('val')
        bus.connect('tester.val', 'mynode.val')

        dt = tester.publish('val', 21)
        ch = node.update()
        self.assertEqual(ch, 'val')
        self.assertEqual(node.time, dt)
        self.assertEqual(node.received, 42)

    def test_publish_from_async(self):
        class AsyncSenderNode(AsyncNode):
            def __init__(self, config, bus):
                super().__init__(config, bus)
                bus.register('out')

            async def on_trigger(self, data):
                await asyncio.sleep(0.01)
                self.publish('out', data + 10)

        bus = Bus(logger=create_mock_logger())
        sender = AsyncSenderNode(config={}, bus=bus.handle('sender'))
        receiver = bus.handle('receiver')
        tester = bus.handle('tester')
        tester.register('trigger')
        bus.connect('tester.trigger', 'sender.trigger')
        bus.connect('sender.out', 'receiver.out')

        sender.start()
        tester.publish('trigger', 5)

        __, channel, data = receiver.listen()
        self.assertEqual(channel, 'out')
        self.assertEqual(data, 15)

        sender.request_stop()
        sender.join(timeout=1.0)
        self.assertFalse(sender.is_alive())


if __name__ == '__main__':
    unittest.main()

# vim: expandtab sw=4 ts=4
