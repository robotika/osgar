import unittest
from unittest.mock import MagicMock
from queue import Queue
from datetime import timedelta

from osgar.drivers.bus import (BusHandler, BusShutdownException,
                               LogBusHandler, LogBusHandlerInputsOnly,
                               serialize)


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

    def test_publish_serialization(self):
        logger = MagicMock()
        logger.register = MagicMock(return_value=1)
        bus = BusHandler(logger, out={'position':[]})
        bus.publish('position', (-123, 456))
        logger.write.assert_called_once_with(1, b'(-123, 456)')

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

    def test_log_bus_handler(self):
        log = MagicMock()
        log_data = [
            (timedelta(microseconds=10), 1, b'(1,2)'),
            (timedelta(microseconds=11), 1, b'(3,4,5)'),
            (timedelta(microseconds=30), 2, b'[8,9]'),
        ]
        log.read_gen = MagicMock(return_value=iter(log_data))
        inputs = {1:'raw'}
        outputs = {2:'can'}
        bus = LogBusHandler(log, inputs, outputs)
        bus.listen()
        with self.assertRaises(AssertionError) as e:
            bus.publish('can', b'parsed data')
        self.assertEqual(str(e.exception), "(b'parsed data', [8, 9])")

    def test_wrong_publish_channel(self):
        log = MagicMock()
        log_data = [
            (timedelta(microseconds=10), 1, b'(1,2)'),
            (timedelta(microseconds=30), 2, b'[8,9]'),
            (timedelta(microseconds=35), 3, b'[8,9]'),
        ]
        log.read_gen = MagicMock(return_value=iter(log_data))
        inputs = {1:'raw'}
        outputs = {2:'can', 3:'can2'}
        bus = LogBusHandler(log, inputs, outputs)
        bus.listen()
        with self.assertRaises(AssertionError) as e:
            bus.publish('can2', [8, 9])
        self.assertEqual(str(e.exception), "('can2', 'can')")

        with self.assertRaises(AssertionError) as e:
            bus.publish('can3', [1, 2])
        self.assertEqual(str(e.exception), "('can3', dict_values(['can', 'can2']))")

    def test_log_bus_handler_inputs_onlye(self):
        log = MagicMock()
        log_data = [
            (timedelta(microseconds=10), 1, b'(1,2)'),
            (timedelta(microseconds=11), 1, b'(3,4,5)'),
            (timedelta(microseconds=30), 2, b'[8,9]'),
        ]
        log.read_gen = MagicMock(return_value=iter(log_data))
        inputs = {1:'raw'}
        bus = LogBusHandlerInputsOnly(log, inputs)
        self.assertEqual(bus.listen(), (timedelta(microseconds=10), 'raw', (1, 2)))
        bus.publish('new_channel', b'some data')
        self.assertEqual(bus.listen(), (timedelta(microseconds=11), 'raw', (3, 4, 5)))

    def test_serialization(self):
            self.assertEqual(serialize(b'\x01\x02'), b'\x01\x02')
            position = (51749517, 180462688)
            self.assertEqual(serialize(position), b'(51749517, 180462688)')
            self.assertEqual(serialize((123.4, 'Hi')), b"(123.4, 'Hi')")

# vim: expandtab sw=4 ts=4
