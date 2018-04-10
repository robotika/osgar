"""
  Internal bus for communication among modules
"""
from queue import Queue
from ast import literal_eval


class BusShutdownException(Exception):
    pass


def serialize(data):
    try:
        bytes_data = data.tobytes()
    except AttributeError:
        if isinstance(data, bytes):
            bytes_data = data
        else:
            bytes_data = bytes(str(data), encoding='ascii')
    return bytes_data


class BusHandler:
    def __init__(self, logger, name='', out={}):
        self.logger = logger
        self.queue = Queue()
        self.name = name
        self.out = out
        self.stream_id = {}
        for publish_name in out.keys():
            idx = self.logger.register('.'.join([self.name, publish_name]))
            self.stream_id[publish_name] = idx
        self._is_alive = True

    def publish(self, channel, data):
        with self.logger.lock:
            stream_id = self.stream_id[channel]  # local maping of indexes
            timestamp = self.logger.write(stream_id, serialize(data))
            for queue, input_channel in self.out[channel]:
                queue.put((timestamp, input_channel, data))

    def listen(self):
        packet = self.queue.get()
        if packet is None:
            raise BusShutdownException()
        timestamp, channel, data = packet
        return timestamp, channel, data

    def is_alive(self):
        return self._is_alive

    def shutdown(self):
        self._is_alive = False
        self.queue.put(None)


class LogBusHandler:
    def __init__(self, log, inputs, outputs):
        self.reader = log.read_gen(list(inputs.keys()) + list(outputs.keys()))
        self.inputs = inputs
        self.outputs = outputs
        self.buffer_queue = Queue()

    def listen(self):
        if self.buffer_queue.empty():
            dt, stream_id, data = next(self.reader)
        else:
            dt, stream_id, data = self.buffer_queue.get()
        channel = self.inputs[stream_id]
        return dt, channel, literal_eval(data.decode('ascii'))

    def publish(self, channel, data):
        assert channel in self.outputs.values(), (channel, self.outputs.values())
        dt, stream_id, raw_data = next(self.reader)
        while stream_id not in self.outputs:
            assert stream_id in self.inputs, stream_id
            self.buffer_queue.put((dt, stream_id, raw_data))
            dt, stream_id, raw_data = next(self.reader)
        assert channel == self.outputs[stream_id], (channel, self.outputs[stream_id])  # wrong channel
        ref_data = literal_eval(raw_data.decode('ascii'))
        assert data == ref_data, (data, ref_data)


class LogBusHandlerInputsOnly:
    def __init__(self, log, inputs):
        self.reader = log.read_gen(inputs.keys())
        self.inputs = inputs

    def listen(self):
        dt, stream_id, data = next(self.reader)
        channel = self.inputs[stream_id]
        return dt, channel, literal_eval(data.decode('ascii'))

    def publish(self, channel, data):
        pass


if __name__ == "__main__":
    pass

# vim: expandtab sw=4 ts=4
