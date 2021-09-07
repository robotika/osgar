import contextlib
from threading import Thread
import zmq
import msgpack

from osgar.bus import BusShutdownException


class PushBind:

    def __init__(self, config, bus):
        bus.register()
        self.endpoint = config.get('endpoint', 'tcp://*:5555')
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
        socket.setsockopt(zmq.LINGER, 100)  # milliseconds
        socket.bind(self.endpoint)

        try:
            with contextlib.closing(socket):
                while True:
                    dt, channel, data = self.bus.listen()
                    raw = msgpack.packb(data, use_bin_type=True)
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
