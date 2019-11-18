import unittest
from unittest.mock import MagicMock
from queue import Queue
from datetime import timedelta

from osgar.bus import (BusHandler, BusShutdownException,
                       LogBusHandler, LogBusHandlerInputsOnly,
                       almost_equal)

from osgar.lib.serialize import serialize, deserialize


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
        logger.write.assert_called_once_with(1, b'\xc4\x18some binary data 2nd try')

    def test_publish_serialization(self):
        logger = MagicMock()
        logger.register = MagicMock(return_value=1)
        bus = BusHandler(logger, out={'position':[]})
        bus.publish('position', (-123, 456))
        logger.write.assert_called_once_with(1, b'\x92\xd0\x85\xcd\x01\xc8')

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
        logger.write.assert_called_once_with(1, b'\xc4\x08bin data')

    def test_alive(self):
        logger = MagicMock()
        bus = BusHandler(logger, out={})
        self.assertTrue(bus.is_alive())
        bus.shutdown()
        self.assertFalse(bus.is_alive())

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
        self.assertEqual(str(e.exception), "(b'parsed data', [8, 9], datetime.timedelta(0, 0, 30))")

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
        self.assertEqual(str(e.exception), "('can2', 'can', datetime.timedelta(0, 0, 30))")

        with self.assertRaises(AssertionError) as e:
            bus.publish('can3', [1, 2])
        self.assertEqual(str(e.exception), "('can3', dict_values(['can', 'can2']))")

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
        bus = BusHandler(log)
        bus.report_error(KeyError(123))
        log.write.assert_called_once_with(0, b"{'error': '123'}")

    def test_publish_time(self):
        logger = MagicMock()
        logger.register = MagicMock(return_value=1)
        logger.write = MagicMock(return_value=timedelta(123))
        bus = BusHandler(logger, out={'stdout':[]})
        self.assertEqual(bus.publish('stdout', 'hello world!'), timedelta(123))

    def test_bus_sleep(self):
        logger = MagicMock()
        bus = BusHandler(logger, out={})
        bus.sleep(0.1)

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

    def test_compression(self):
        logger = MagicMock()
        logger.register = MagicMock(return_value=1)
        bus = BusHandler(logger, out={'raw':[]}, name='tcp_point_data')
        # TODO proper definition what should be compressed - now hardcoded to name "tcp_point_data"
        self.assertTrue(bus.compressed_output)
        bus.publish('raw', b'\00'*10000)
        logger.write.assert_called_once_with(1, 
            b'x\x9c\xed\xc1\x01\r\x00\x00\x08\x03 #\xbc\x85\xfdC\xd8\xcb\x1e\x1fp\x9b' +
            b'\x01\x00\x00\x00\x00\x00\x00\x00\x00\x80\x02\x0f\x9f\xba\x00\xfd')

    def test_logged_func_log(self):
        def func_to_be_logged(a):
            return a ** 2
        logger = MagicMock()
        logger.register = MagicMock(return_value=1)
        logger.write = MagicMock(return_value=timedelta(123))
        bus = BusHandler(logger, 'test')
        logged = bus.logged(func_to_be_logged)
        self.assertEqual(func_to_be_logged(2), logged(2))
        logger.register.assert_called_with('test.func_to_be_logged')
        logger.write.assert_called_once_with(1, serialize([[2],{},4]))

    def test_logged_func_replay(self):
        def func_to_be_logged(a):
            return a ** 2
        log = [
            (0, 0, serialize([[2], {}, 4])),
        ]
        bus = LogBusHandler(iter(log), [], [])
        logged = bus.logged(func_to_be_logged)
        self.assertEqual(func_to_be_logged(2), logged(2))

    def test_logged_class_log(self):
        class A:
            def __init__(self, aa):
                self.aa = aa
            def doSomething(self, b):
                return self.aa * b

        logger = MagicMock()
        logger.register = MagicMock(return_value=1)
        logger.write = MagicMock(return_value=timedelta(123))
        bus = BusHandler(logger, 'test')
        a = A(4)
        la = bus.logged(A, 4)

        self.assertEqual(a.doSomething(2), la.doSomething(2))
        logger.register.assert_called_once_with('test.A')
        logger.write.assert_called_once_with(1, serialize([[2],{},8]))

    def test_logged_class_replay(self):
        class A:
            def __init__(self, aa):
                self.aa = aa
            def doSomething(self, b):
                return self.aa * b
        log = [
            (0, 0, serialize([[2], {}, 8])),
        ]
        bus = LogBusHandler(iter(log), [], [])
        a = A(4)
        la = bus.logged(A, 4)

        self.assertEqual(a.doSomething(2), la.doSomething(2))


# vim: expandtab sw=4 ts=4
