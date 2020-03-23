import unittest
import threading
import time

from unittest.mock import MagicMock
from datetime import timedelta

from osgar.bus import (Bus, BusShutdownException,
                       LogBusHandler, LogBusHandlerInputsOnly,
                       almost_equal)

from osgar.lib.serialize import serialize


class BusHandlerTest(unittest.TestCase):

    def test_publish(self):
        logger = MagicMock()
        bus = Bus(logger)
        handle = bus.handle('test')
        with self.assertRaises(KeyError):
            handle.publish('raw', b'some binary data')

        logger = MagicMock()
        logger.register = MagicMock(return_value=1)
        bus = Bus(logger)
        handle = bus.handle('test')
        handle.register('raw')
        handle.publish('raw', b'some binary data 2nd try')
        logger.write.assert_called_once_with(1, b'\xc4\x18some binary data 2nd try')

    def test_publish_serialization(self):
        logger = MagicMock()
        logger.register = MagicMock(return_value=1)
        bus = Bus(logger)
        handle = bus.handle('test')
        handle.register('position')
        handle.publish('position', (-123, 456))
        logger.write.assert_called_once_with(1, b'\x92\xd0\x85\xcd\x01\xc8')

    def test_listen(self):
        logger = MagicMock()
        bus = Bus(logger)
        handler = bus.handle('test')
        handler.queue.put((1, 2, b"bin data"))
        self.assertEqual(handler.listen(), (1, 2, b"bin data"))

    def test_two_modules(self):
        logger = MagicMock()
        bus = Bus(logger)
        handler2 = bus.handle('2')
        handler1 = bus.handle('1')
        handler1.register('raw')
        bus.connect('1.raw', '2.42')

        logger.write = MagicMock(return_value=123)
        handler1.publish('raw', b"Hello!")

        self.assertEqual(handler2.listen(), (123, '42', b"Hello!"))

    def test_shutdown(self):
        logger = MagicMock()
        bus = Bus(logger)
        handler = bus.handle('test')
        handler.shutdown()
        with self.assertRaises(BusShutdownException):
            handler.listen()

    def test_named_publish(self):
        logger = MagicMock()
        logger.register = MagicMock(return_value=1)
        bus = Bus(logger)
        handle = bus.handle('gps_serial')
        handle.register('raw')
        handle.publish('raw', b'bin data')
        logger.write.assert_called_once_with(1, b'\xc4\x08bin data')

    def test_alive(self):
        logger = MagicMock()
        bus = Bus(logger)
        handle = bus.handle('test')
        self.assertTrue(handle.is_alive())
        handle.shutdown()
        self.assertFalse(handle.is_alive())

    def test_log_bus_handler(self):
        log_data = [
            (timedelta(microseconds=10), 1, serialize(b'(1,2)')),
            (timedelta(microseconds=11), 1, serialize(b'(3,4,5)')),
            (timedelta(microseconds=30), 2, serialize([8, 9])),
        ]
        log = iter(log_data)
        inputs = {1:'raw'}
        outputs = {2:'can'}
        bus = LogBusHandler(log, inputs, outputs)
        bus.listen()
        with self.assertRaises(AssertionError) as e:
            bus.publish('can', b'parsed data')
        self.assertEqual(e.exception.args[0], (b'parsed data', [8, 9], timedelta(0, 0, 30)))

    def test_wrong_publish_channel(self):
        log_data = [
            (timedelta(microseconds=10), 1, serialize(b'(1,2)')),
            (timedelta(microseconds=30), 2, serialize(b'[8,9]')),
            (timedelta(microseconds=35), 3, serialize(b'[8,9]')),
        ]
        log = iter(log_data)
        inputs = {1:'raw'}
        outputs = {2:'can', 3:'can2'}
        bus = LogBusHandler(log, inputs, outputs)
        bus.listen()
        with self.assertRaises(AssertionError) as e:
            bus.publish('can2', [8, 9])
        self.assertEqual(e.exception.args[0], ('can2', 'can', timedelta(0, 0, 30)))

        with self.assertRaises(AssertionError) as e:
            bus.publish('can3', [1, 2])
        self.assertEqual(e.exception.args[0], ('can3', ('can', 'can2')))

    def test_log_bus_handler_inputs_onlye(self):
        log_data = [
            (timedelta(microseconds=10), 1, serialize([1, 2])),
            (timedelta(microseconds=11), 1, serialize([3, 4, 5])),
            (timedelta(microseconds=30), 2, serialize([8, 9])),
        ]
        log = iter(log_data)
        inputs = {1:'raw'}
        bus = LogBusHandlerInputsOnly(log, inputs)
        self.assertEqual(bus.listen(), (timedelta(microseconds=10), 'raw', [1, 2]))
        bus.publish('new_channel', b'some data')
        self.assertEqual(bus.listen(), (timedelta(microseconds=11), 'raw', [3, 4, 5]))

    def test_report_error(self):
        log = MagicMock()
        bus = Bus(log)
        handle = bus.handle('test')
        handle.report_error(KeyError(123))
        log.write.assert_called_once_with(0, b"{'error': '123'}")

    def test_publish_time(self):
        logger = MagicMock()
        logger.register = MagicMock(return_value=1)
        logger.write = MagicMock(return_value=timedelta(123))
        bus = Bus(logger)
        handle = bus.handle('test')
        handle.register('stdout')
        self.assertEqual(handle.publish('stdout', 'hello world!'), timedelta(123))

    def test_bus_sleep(self):
        logger = MagicMock()
        bus = Bus(logger)
        handle = bus.handle('test')
        handle.sleep(0.1)

        bus = LogBusHandler(logger, inputs={}, outputs={})
        bus.sleep(0.1)

        bus = LogBusHandlerInputsOnly(logger, inputs={})
        bus.sleep(0.1)

    def test_almost_equal(self):
        self.assertTrue(almost_equal(-0.27335569599868276, -0.2733556959986828))
        self.assertFalse(almost_equal(-0.27, 0.42))
        self.assertTrue(almost_equal(
            [[-0.27335569599868276, 2.625292235055242, -0.14962045778119396], [0.7263869464850868, 0.0064504746910471825, -0.007071867505754647, 0.6873936102882865]], 
            [[-0.2733556959986828,  2.625292235055242, -0.14962045778119396], [0.7263869464850868, 0.0064504746910471825, -0.007071867505754647, 0.6873936102882865]]
            ))
        self.assertFalse(almost_equal([1.23], []))
        self.assertFalse(almost_equal([-0.27], [0.42]))

    def test_sleep(self):
        bus = Bus(MagicMock())
        handle = bus.handle('test')
        interval = 0.5
        def sleep():
            handle.sleep(interval)
        t = threading.Thread(target=sleep, daemon=True)
        start = time.monotonic()
        t.start()
        t.join(10)
        end = time.monotonic()
        self.assertFalse(t.is_alive())
        self.assertGreaterEqual(end-interval, start)

    def test_interruptible_sleep(self):
        bus = Bus(MagicMock())
        handle = bus.handle('test')
        def sleep():
            handle.sleep(10)
        t = threading.Thread(target=sleep, daemon=True)
        t.start()
        handle.shutdown()
        t.join(0.01)
        self.assertFalse(t.is_alive())

# vim: expandtab sw=4 ts=4
