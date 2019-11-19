"""
  Internal bus for communication among modules
"""
import time
import zlib
import functools
import types

from queue import Queue
from datetime import timedelta
from collections import deque

from osgar.lib.serialize import serialize, deserialize


# restrict replay time from given input
ASSERT_QUEUE_DELAY = timedelta(seconds=.1)

def almost_equal(data, ref_data):
    if isinstance(data, float) and isinstance(ref_data, float):
        return abs(data - ref_data) < 1e-6
    if isinstance(data, list) and isinstance(ref_data, list):
        if len(data) != len(ref_data):
            return False
        for a, b in zip(data, ref_data):
            if not(almost_equal(a, b)):
                return False
        return True
    # default - use exact match
    return data == ref_data


class BusShutdownException(Exception):
    pass


def _logged_func(func, log_write):
    @functools.wraps(func)
    def logged(*args, **kwargs):
        ret = func(*args, **kwargs)
        data = serialize([args, kwargs, ret])
        log_write(data)
        return ret
    return logged


def _replayed_func(func, log_read):
    @functools.wraps(func)
    def replayed(*args, **kwargs):
        dt, stream_id, data = log_read()
        log_args, log_kwargs, log_ret = deserialize(data)
        assert tuple(log_args) == args, (tuple(log_args), args)
        assert log_kwargs == kwargs
        return log_ret
    return replayed


def _logged_class(klass, log_write, *args, **kwargs):
    orig = klass(*args, **kwargs)
    class Logged:
        def __getattr__(self, attr):
            orig_attr = getattr(orig, attr)
            #print(attr, orig_attr, type(orig_attr))
            if isinstance(orig_attr, (types.MethodType, types.BuiltinMethodType)):
                logged = _logged_func(orig_attr, log_write)
                # TODO: record method name along as well (for debugging)
                setattr(self, attr, logged)
                return logged
            raise RuntimeError("only methods can be accessed on logged class")
    return Logged()


def _replayed_class(klass, log_read, *args, **kwargs):
    class Replayed:
        def __getattr__(self, attr):
            return self._replay

        def _replay(self, *args, **kwargs):
            dt, stream_id, data = log_read()
            print(dt, stream_id, data[:10])
            log_args, log_kwargs, log_ret = deserialize(data)
            assert tuple(log_args) == args, (tuple(log_args), args)
            assert log_kwargs == kwargs
            return log_ret

    return Replayed()


class BusHandler:
    def __init__(self, logger, name='', out={}, slots={}):
        self.logger = logger
        self.queue = Queue()
        self.name = name
        self.out = out
        self.slots = slots
        self.stream_id = {}
        for publish_name in out.keys():
            idx = self.logger.register('.'.join([self.name, publish_name]))
            self.stream_id[publish_name] = idx
        self._is_alive = True
        self.compressed_output = (name == 'tcp_point_data')  # hack

    def publish(self, channel, data):
        with self.logger.lock:
            stream_id = self.stream_id[channel]  # local maping of indexes
            if self.compressed_output:
                to_write = zlib.compress(serialize(data))
            else:
                to_write = serialize(data)
            timestamp = self.logger.write(stream_id, to_write)

            for queue, input_channel in self.out[channel]:
                queue.put((timestamp, input_channel, data))
            for slot in self.slots.get(channel, []):
                slot(timestamp, data)
        return timestamp

    def listen(self):
        packet = self.queue.get()
        if packet is None:
            raise BusShutdownException()
        timestamp, channel, data = packet
        return timestamp, channel, data

    def logged(self, what, *args, **kwargs):
        stream_id = self.logger.register(f"{self.name}.{what.__name__}")
        log_write = functools.partial(self.logger.write, stream_id)
        if isinstance(what, types.FunctionType):
            return _logged_func(what, log_write)
        return _logged_class(what, log_write, *args, **kwargs)

    def sleep(self, secs):
        time.sleep(secs)

    def is_alive(self):
        return self._is_alive

    def shutdown(self):
        self._is_alive = False
        self.queue.put(None)

    def report_error(self, err):
        with self.logger.lock:
            self.logger.write(0, bytes(str({'error': str(err)}),
                                       encoding='ascii'))


class LogBusHandler:
    def __init__(self, log, inputs, outputs):
        self.reader = log
        self.inputs = inputs
        self.outputs = outputs
        self.buffer_queue = deque()
        self.max_delay = timedelta()
        self.max_delay_timestamp = timedelta()

    def listen(self):
        while True:
            if len(self.buffer_queue) == 0:
                dt, stream_id, bytes_data = next(self.reader)
            else:
                dt, stream_id, bytes_data = self.buffer_queue.popleft()
            channel = self.inputs[stream_id]
            data = deserialize(bytes_data)
            if channel.startswith("slot_"):
                getattr(self.node, channel)(dt, data)
            else:
                break
        return dt, channel, data

    def publish(self, channel, data):
        assert channel in self.outputs.values(), (channel, self.outputs.values())
        dt, stream_id, bytes_data = next(self.reader)
        while stream_id not in self.outputs:
            input_name = self.inputs[stream_id]
            if input_name.startswith("slot_"):
                data = deserialize(bytes_data)
                getattr(self.node, input_name)(dt, data)
            else:
                self.buffer_queue.append((dt, stream_id, bytes_data))
            dt, stream_id, bytes_data = next(self.reader)
        assert channel == self.outputs[stream_id], (channel, self.outputs[stream_id], dt)  # wrong channel
        if len(self.buffer_queue) > 0:
            delay = dt - self.buffer_queue[0][0]
            if delay > self.max_delay:
                self.max_delay = delay
                self.max_delay_timestamp = dt
            if delay > ASSERT_QUEUE_DELAY:
                print(dt, "maximum delay overshot:", delay)
        ref_data = deserialize(bytes_data)
        assert almost_equal(data, ref_data), (data, ref_data, dt)
        return dt

    def logged(self, what, *args, **kwargs):
        #stream_id = self.reader.register(f"{self.name}.{what.__name__}")
        if isinstance(what, types.FunctionType):
            return _replayed_func(what, self.reader.__next__)
        return _replayed_class(what, self.reader.__next__, *args, **kwargs)

    def sleep(self, secs):
        pass

    def is_alive(self):
        return True


class LogBusHandlerInputsOnly:
    def __init__(self, log, inputs):
        self.reader = log
        self.inputs = inputs
        self.time = timedelta(0)

    def listen(self):
        dt, stream_id, bytes_data = next(self.reader)
        self.time = dt
        channel = self.inputs[stream_id]
        data = deserialize(bytes_data)
        if channel.startswith("slot_"):
            channel = channel[len('slot_'):]  # workaround for non-slot run
        return dt, channel, data

    def publish(self, channel, data):
        return self.time

    def sleep(self, secs):
        pass


if __name__ == "__main__":
    pass

# vim: expandtab sw=4 ts=4
