import contextlib

from threading import Thread

import zmq

from osgar.bus import BusShutdownException
from osgar.lib.serialize import serialize


class Push:

    def __init__(self, config, bus):
        bus.register()
        self.is_bind_set = config.get("bind", False)
        self.endpoint = config.get('endpoint', 'tcp://127.0.0.1:5566')
        self.timeout = config.get('timeout', 1) # default send timeout 1s, effective on full queue only
        self.thread = Thread(target=self.run)
        self.thread.name = bus.name
        self.bus = bus

    def start(self):
        self.thread.start()

    def join(self, timeout=None):
        self.thread.join(timeout=timeout)

    def run(self):
        context = zmq.Context.instance()
        socket = context.socket(zmq.PUSH)

        if self.is_bind_set:
            socket.setsockopt(zmq.LINGER, 100)  # milliseconds
            socket.bind(self.endpoint)
        else:
            # https://stackoverflow.com/questions/7538988/zeromq-how-to-prevent-infinite-wait
            socket.SNDTIMEO = int(self.timeout * 1000)  # convert to milliseconds
            socket.LINGER = 100 #milliseconds
            socket.connect(self.endpoint)

        try:
            with contextlib.closing(socket):
                while True:
                    dt, channel, data = self.bus.listen()
                    raw = serialize(data)
                    socket.send_multipart([bytes(channel, 'ascii'), raw])
        except zmq.ZMQError as e:
            if e.errno == zmq.EAGAIN:
                pass #TODO log timeout
            else:
                pass #TODO log other error
        except BusShutdownException:
            pass

    def request_stop(self):
        self.bus.shutdown()
