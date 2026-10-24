import socket
import time
import unittest
from datetime import timedelta
from unittest.mock import MagicMock

from osgar.bus import Bus
from osgar.drivers.asyncecho import AsyncEcho


def create_mock_logger():
    logger = MagicMock()
    logger.write = MagicMock(return_value=timedelta(seconds=1))
    return logger


class AsyncEchoTest(unittest.TestCase):

    def test_startup_and_shutdown(self):
        bus = Bus(logger=create_mock_logger())
        config = {'host': '127.0.0.1', 'port': 0}
        device = AsyncEcho(config=config, bus=bus.handle('asyncecho'))
        device.start()
        self.assertTrue(device.ready.wait(timeout=2.0))
        self.assertGreater(device.port, 0)
        device.request_stop()
        device.join(timeout=1.0)
        self.assertFalse(device.is_alive())

    def test_client_to_bus(self):
        bus = Bus(logger=create_mock_logger())
        config = {'host': '127.0.0.1', 'port': 0}
        device = AsyncEcho(config=config, bus=bus.handle('asyncecho'))
        tester = bus.handle('tester')
        bus.connect('asyncecho.raw', 'tester.raw')

        device.start()
        self.assertTrue(device.ready.wait(timeout=2.0))

        client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client.connect(('127.0.0.1', device.port))
        client.sendall(b'ping\n')

        timestamp, channel, data = tester.listen()
        self.assertEqual(channel, 'raw')
        self.assertEqual(data, b'ping\n')

        client.close()
        device.request_stop()
        device.join(timeout=1.0)
        self.assertFalse(device.is_alive())

    def test_bus_to_client(self):
        bus = Bus(logger=create_mock_logger())
        config = {'host': '127.0.0.1', 'port': 0}
        device = AsyncEcho(config=config, bus=bus.handle('asyncecho'))
        tester = bus.handle('tester')
        tester.register('raw')
        bus.connect('tester.raw', 'asyncecho.raw')

        device.start()
        self.assertTrue(device.ready.wait(timeout=2.0))

        client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client.connect(('127.0.0.1', device.port))
        client.settimeout(2.0)

        # Allow connection to be accepted
        time.sleep(0.05)

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
