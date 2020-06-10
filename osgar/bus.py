"""
  Internal bus for communication among modules
"""
import inspect
import threading  # Event

from queue import Queue
from datetime import timedelta
from collections import deque

from osgar.lib.serialize import serialize, deserialize, compress


# restrict replay time from given input
ASSERT_QUEUE_DELAY = timedelta(seconds=.1)

def almost_equal(data, ref_data):
    if isinstance(data, float) and isinstance(ref_data, float):
        if data == ref_data:
            return True  # handling float('inf')
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


class Bus:
    def __init__(self, logger):
        self.logger = logger
        self.handles = {}

    def handle(self, name):
        return self.handles.setdefault(name, _BusHandler(self.logger, name))

    def connect(self, sender, receiver, modules=None):
        sender, output = sender.split('.')
        receiver, input = receiver.split('.')
        self.handles[sender].connect(output, self.handles[receiver], input, modules)

    def delays(self):
        for name, handle in self.handles.items():
            if handle.max_delay > ASSERT_QUEUE_DELAY:
                yield name, handle.max_delay, handle.max_delay_timestamp


class _BusHandler:
    def __init__(self, logger, name):
        self.logger = logger
        self.queue = Queue()
        self.name = name
        self.out = {}
        self.slots = {}
        self.stream_id = {}
        self.finished = threading.Event()
        self.no_output = set()
        self.compressed_output = set()
        self._time = None
        self.max_delay = timedelta()
        self.max_delay_timestamp = timedelta()

    def register(self, *outputs):
        for name_and_type in outputs:
            o = name_and_type.split(':')[0]  # keep only prefixes
            if o in self.stream_id:
                continue
            idx = self.logger.register(f'{self.name}.{o}')
            self.stream_id[o] = idx
            if name_and_type.endswith(':null'):
                self.no_output.add(idx)
            if name_and_type.endswith(':gz'):
                self.compressed_output.add(idx)
            self.out[o] = []
            self.slots[o] = []

    def connect(self, output, receiver, input, modules):
        if output not in self.out:
            #raise RuntimeError(f"'{self.name}' does not have '{output}' as output.")
            print(f"'{self.name}' does not have '{output}' as output.")
            return  # TODO parametrize by -F (force) or --ignore-missing_inputs??
        if input.startswith('slot_'):
            assert modules is not None
            assert receiver.name in modules
            self.slots[output].append(getattr(modules[receiver.name], input))
        else:
            self.out[output].append((receiver.queue, input))

    def publish(self, channel, data):
        with self.logger.lock:
            stream_id = self.stream_id[channel]  # local maping of indexes
            if stream_id in self.no_output:
                to_write = serialize(None)  # i.e. at least there will be timestamp record
            else:
                to_write = serialize(data)
                if stream_id in self.compressed_output:
                    to_write = compress(data)
            timestamp = self.logger.write(stream_id, to_write)

            if self._time is not None:
                delay = timestamp - self._time
                if delay > self.max_delay:
                    self.max_delay = delay
                    self.max_delay_timestamp = timestamp
                if delay > ASSERT_QUEUE_DELAY:
                    caller = inspect.currentframe().f_back.f_code.co_name
                    self.report_error(delay=delay.total_seconds(), channel=channel, caller=caller)

            for queue, input_channel in self.out[channel]:
                queue.put((timestamp, input_channel, data))
            for slot in self.slots.get(channel, []):
                slot(timestamp, data)
        return timestamp

    def listen(self):
        packet = self.queue.get()
        if packet is None:
            raise BusShutdownException()
        self._time, channel, data = packet
        return self._time, channel, data

    def sleep(self, secs):
        self.finished.wait(secs)

    def is_alive(self):
        return not self.finished.is_set()

    def shutdown(self):
        self.finished.set()
        self.queue.put(None)

    def report_error(self, **err):
        with self.logger.lock:
            data = dict(name=self.name)
            err.pop('name', None)
            data.update(err)
            self.logger.write(0, bytes(str(data), encoding='ascii'))


class LogBusHandler:
    def __init__(self, log, inputs, outputs):
        self.reader = log
        self.inputs = inputs
        self.outputs = outputs
        self.buffer_queue = deque()
        self.max_delay = timedelta()
        self.max_delay_timestamp = timedelta()
        self.finished = threading.Event()

    def register(self, *outputs):
        for o in outputs:
            # ignore :gz and :null modifiers
            assert o.split(':')[0] in self.outputs.values(), (o, self.outputs)

    def listen(self):
        while True:
            if self.finished.is_set():
                raise BusShutdownException
            if len(self.buffer_queue) == 0:
                dt, stream_id, bytes_data = next(self.reader)
            else:
                dt, stream_id, bytes_data = self.buffer_queue.popleft()
            if stream_id not in self.inputs:
                error = f"unexpected input (stream {stream_id}, known inputs {list(self.inputs.keys())})"
                raise RuntimeError(error)
            channel = self.inputs[stream_id]
            data = deserialize(bytes_data)
            if channel.startswith("slot_"):
                getattr(self.node, channel)(dt, data)
            else:
                break
        return dt, channel, data

    def publish(self, channel, data):
        assert channel in self.outputs.values(), (channel, tuple(self.outputs.values()))
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

    def sleep(self, secs):
        pass

    def report_error(self, err):
        print(self.time, err)


class LogBusHandlerInputsOnly:
    def __init__(self, log, inputs):
        self.reader = log
        self.inputs = inputs
        self.time = timedelta(0)

    def register(self, *outputs):
        pass

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

    def report_error(self, err):
        print(self.time, err)


if __name__ == "__main__":
    pass

# vim: expandtab sw=4 ts=4
