import unittest
from datetime import timedelta
from unittest.mock import MagicMock

from osgar.bus import Bus
from osgar.drivers.asyncecho import AsyncEcho
from examples.asyncecho.asyncecho_app import EchoApp
from osgar.drivers.test_asyncecho import connect_socketpair


def create_mock_logger():
    logger = MagicMock()
    logger.write = MagicMock(return_value=timedelta(seconds=1))
    return logger


class ExampleAsyncEchoTest(unittest.TestCase):

    def test_echo_app_end_to_end(self):
        bus = Bus(logger=create_mock_logger())
        client = connect_socketpair(self)
        config_echo = {}
        echo_driver = AsyncEcho(config=config_echo, bus=bus.handle('echo'))
        app = EchoApp(config={}, bus=bus.handle('app'))
        def stop():
            for node in (echo_driver, app):
                node.force_stop()
                node.join(2)
        self.addCleanup(stop)

        bus.connect('echo.raw', 'app.raw')
        bus.connect('app.raw', 'echo.raw')

        echo_driver.start()
        app.start()

        self.assertTrue(echo_driver.ready.wait(timeout=2.0))

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
