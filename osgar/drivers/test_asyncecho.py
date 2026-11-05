import socket
import asyncio
import unittest
from datetime import timedelta
from unittest.mock import MagicMock, patch

from osgar.bus import Bus
from osgar.drivers.asyncecho import AsyncEcho


def create_mock_logger():
    logger = MagicMock()
    logger.write = MagicMock(return_value=timedelta(seconds=1))
    return logger

def connect_socketpair(test):
    """Run real asyncio streams without binding a TCP listener in CI."""
    connection, client = socket.socketpair()
    client.settimeout(2)
    test.addCleanup(connection.close)
    test.addCleanup(client.close)

    class Server:
        sockets = []

        async def __aenter__(self):
            return self

        async def __aexit__(self, *args):
            pass

    async def start_server(callback, host, port):
        reader, writer = await asyncio.open_connection(sock=connection)
        asyncio.create_task(callback(reader, writer))
        # Wait until the handler registers the client, avoiding timing sleeps.
        await asyncio.sleep(0)
        return Server()

    mock = patch('osgar.drivers.asyncecho.asyncio.start_server', side_effect=start_server)
    mock.start()
    test.addCleanup(mock.stop)
    return client


class AsyncEchoTest(unittest.TestCase):

    def start_device(self, bus):
        device = AsyncEcho(config={}, bus=bus.handle('asyncecho'))
        def stop():
            device.force_stop()
            device.join(2)
        self.addCleanup(stop)
        device.start()
        self.assertTrue(device.ready.wait(timeout=2))
        self.assertIsNotNone(device.server)
        return device

    def test_startup_and_shutdown(self):
        bus = Bus(logger=create_mock_logger())
        connect_socketpair(self)
        device = self.start_device(bus)
        device.request_stop()
        device.join(timeout=1.0)
        self.assertFalse(device.is_alive())

    def test_client_to_bus(self):
        bus = Bus(logger=create_mock_logger())
        client = connect_socketpair(self)
        bus.handle('asyncecho').register('raw')
        tester = bus.handle('tester')
        bus.connect('asyncecho.raw', 'tester.raw')

        device = self.start_device(bus)
        client.sendall(b'ping\n')

        timestamp, channel, data = tester.queue.get(timeout=2)
        self.assertEqual(channel, 'raw')
        self.assertEqual(data, b'ping\n')

        client.close()
        device.request_stop()
        device.join(timeout=1.0)
        self.assertFalse(device.is_alive())

    def test_bus_to_client(self):
        bus = Bus(logger=create_mock_logger())
        client = connect_socketpair(self)
        bus.handle('asyncecho')
        tester = bus.handle('tester')
        tester.register('raw')
        bus.connect('tester.raw', 'asyncecho.raw')

        device = self.start_device(bus)

        tester.publish('raw', b'telemetry-data\n')

        received = client.recv(1024)
        self.assertEqual(received, b'telemetry-data\n')

        client.close()
        device.request_stop()
        device.join(timeout=1.0)
        self.assertFalse(device.is_alive())


if __name__ == '__main__':
    unittest.main()

# vim: expandtab sw=4 ts=4
