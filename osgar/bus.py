"""
  Internal bus for communication among modules
"""
from datetime import timedelta
from collections import deque

import trio

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


async def BusHandler(logger, name='', out=[]):
    self = _BusHandler(logger, name, out)
    for output_name in out:
        idx = await self.logger.register('.'.join([self.name, output_name]))
        self.stream_id[output_name] = idx
    return self


def connect(node_from, output, node_to, input):
    node_from.out[output].append((node_to.receive_channel, input))


class _BusHandler:

    def __init__(self, logger, name, out):
        self.logger = logger
        self.send_channel, self.receive_channel = trio.open_memory_channel(100)
        self.name = name
        self.out = {output_name: [] for output_name in out}
        self.stream_id = {}
        self._is_alive = True

    async def publish(self, channel, data):
        async with self.logger.lock:
            stream_id = self.stream_id[channel]  # local mapping outputs to indexes in log
            to_write = serialize(data)
            timestamp = await self.logger.write(stream_id, to_write)

            for queue, input_channel in self.out[channel]:
                await queue.send((timestamp, input_channel, data))
        return timestamp

    async def listen(self):
        packet = await  self.receive_channel.receive()
        if packet is None:
            raise BusShutdownException()
        timestamp, channel, data = packet
        return timestamp, channel, data

    async def sleep(self, secs):
        await trio.sleep(secs)

    def is_alive(self):
        return self._is_alive

    async def shutdown(self):
        self._is_alive = False
        await self.send_channel.send(None)

    async def report_error(self, err):
        await self.logger.error(err)


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

    def sleep(self, secs):
        pass


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
