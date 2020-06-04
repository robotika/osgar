import contextlib

from threading import Thread

import zmq

from osgar.bus import BusShutdownException
import osgar.lib.serialize

class Pull:

    def __init__(self, config, bus):
        bus.register(*config['outputs'])
        self.endpoint = config.get('endpoint', 'tcp://127.0.0.1:5565')
        self.timeout = config.get('timeout', 1) # default recv timeout 1s
        self.thread = Thread(target=self.run)
        self.thread.name = bus.name
        self.bus = bus

    def start(self):
        self.thread.start()

    def join(self, timeout=None):
        self.thread.join(timeout=timeout)

    def run(self):
        context = zmq.Context.instance()
        socket = context.socket(zmq.PULL)
        # https://stackoverflow.com/questions/7538988/zeromq-how-to-prevent-infinite-wait
        socket.RCVTIMEO = int(self.timeout * 1000)  # convert to milliseconds
        socket.connect(self.endpoint)

        with contextlib.closing(socket):
            while self.bus.is_alive():
                try:
                    channel, raw = socket.recv_multipart()
                    message = osgar.lib.serialize.deserialize(raw)
                    self.bus.publish(channel.decode('ascii'), message)
                except zmq.error.Again:
                    pass

    def request_stop(self):
        self.bus.shutdown()


