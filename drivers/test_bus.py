import unittest
from unittest.mock import MagicMock
from queue import Queue

from drivers.bus import BusHandler, BusShutdownException


class BusHandlerTest(unittest.TestCase):

    def test_publish(self):
        logger = MagicMock()
        bus = BusHandler(logger)
        with self.assertRaises(KeyError):
            bus.publish(1, b'some binary data')
        logger.write.assert_called_once_with(1, b'some binary data')

        logger = MagicMock()
        bus = BusHandler(logger, out={1:[]})
        bus.publish(1, b'some binary data 2nd try')
        logger.write.assert_called_once_with(1, b'some binary data 2nd try')

    def test_listen(self):
        logger = MagicMock()
        handler = BusHandler(logger)
        handler.queue.put((1, 2, b"bin data"))
        self.assertEqual(handler.listen(), (1, 2, b"bin data"))

    def test_two_modules(self):
        logger = MagicMock()
        handler2 = BusHandler(logger)
        handler1 = BusHandler(logger, out={1:[(handler2.queue, 42)]})

        logger.write = MagicMock(return_value=123)
        handler1.publish(1, b"Hello!")

        self.assertEqual(handler2.listen(), (123, 42, b"Hello!"))

    def test_shutdown(self):
        logger = MagicMock()
        handler = BusHandler(logger)
        handler.shutdown()
        with self.assertRaises(BusShutdownException):
            handler.listen()

# vim: expandtab sw=4 ts=4
