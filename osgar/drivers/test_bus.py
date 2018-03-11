import unittest
from unittest.mock import MagicMock
from queue import Queue

from drivers.bus import BusHandler, BusShutdownException


class BusHandlerTest(unittest.TestCase):

    def test_publish(self):
        logger = MagicMock()
        bus = BusHandler(logger)
        with self.assertRaises(KeyError):
            bus.publish('raw', b'some binary data')

        logger = MagicMock()
        logger.register = MagicMock(return_value=1)
        bus = BusHandler(logger, out={'raw':[]})
        bus.publish('raw', b'some binary data 2nd try')
        logger.write.assert_called_once_with(1, b'some binary data 2nd try')

    def test_listen(self):
        logger = MagicMock()
        handler = BusHandler(logger)
        handler.queue.put((1, 2, b"bin data"))
        self.assertEqual(handler.listen(), (1, 2, b"bin data"))

    def test_two_modules(self):
        logger = MagicMock()
        handler2 = BusHandler(logger)
        handler1 = BusHandler(logger, out={'raw':[(handler2.queue, 42)]})

        logger.write = MagicMock(return_value=123)
        handler1.publish('raw', b"Hello!")

        self.assertEqual(handler2.listen(), (123, 42, b"Hello!"))

    def test_shutdown(self):
        logger = MagicMock()
        handler = BusHandler(logger)
        handler.shutdown()
        with self.assertRaises(BusShutdownException):
            handler.listen()

    def test_named_publish(self):
        logger = MagicMock()
        logger.register = MagicMock(return_value=1)
        bus = BusHandler(logger, name='gps_serial', out={'raw':[]})
        bus.publish('raw', b'bin data')
        logger.write.assert_called_once_with(1, b'bin data')

    def test_alive(self):
        logger = MagicMock()
        bus = BusHandler(logger, out={})
        self.assertTrue(bus.is_alive())
        bus.shutdown()
        self.assertFalse(bus.is_alive())

# vim: expandtab sw=4 ts=4
