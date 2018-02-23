"""
  Internal bus for communication among modules
"""
from queue import Queue


class BusShutdownException(Exception):
    pass


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

    def publish(self, channel, data):
        with self.logger.lock:
            stream_id = self.stream_id[channel]  # local maping of indexes
            timestamp = self.logger.write(stream_id, data)
            for queue, input_channel in self.out[channel]:
                queue.put((timestamp, input_channel, data))

    def listen(self):
        packet = self.queue.get()
        if packet is None:
            raise BusShutdownException()
        timestamp, channel, data = packet
        return timestamp, channel, data

    def shutdown(self):
        self.queue.put(None)


if __name__ == "__main__":
    pass

# vim: expandtab sw=4 ts=4
