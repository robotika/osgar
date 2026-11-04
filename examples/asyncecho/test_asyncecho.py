import socket
import time
import unittest
from datetime import timedelta
from unittest.mock import MagicMock

from osgar.bus import Bus
from osgar.drivers.asyncecho import AsyncEcho
from examples.asyncecho.asyncecho_app import EchoApp


def create_mock_logger():
    logger = MagicMock()
    logger.write = MagicMock(return_value=timedelta(seconds=1))
    return logger


class ExampleAsyncEchoTest(unittest.TestCase):

    def test_echo_app_end_to_end(self):
        bus = Bus(logger=create_mock_logger())
        config_echo = {'host': '127.0.0.1', 'port': 0}
        echo_driver = AsyncEcho(config=config_echo, bus=bus.handle('echo'))
        app = EchoApp(config={}, bus=bus.handle('app'))

        bus.connect('echo.raw', 'app.raw')
        bus.connect('app.raw', 'echo.raw')

        echo_driver.start()
        app.start()

        self.assertTrue(echo_driver.ready.wait(timeout=2.0))

        client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client.connect(('127.0.0.1', echo_driver.port))
        client.settimeout(2.0)

        # Allow TCP handshake to complete
        time.sleep(0.05)

        client.sendall(b'world\n')
        reply = client.recv(1024)
        self.assertEqual(reply, b'ECHO: world\n')

        client.close()
        echo_driver.request_stop()
        app.request_stop()

        echo_driver.join(timeout=1.0)
        app.join(timeout=1.0)

        self.assertFalse(echo_driver.is_alive())
        self.assertFalse(app.is_alive())


if __name__ == '__main__':
    unittest.main()

# vim: expandtab sw=4 ts=4
