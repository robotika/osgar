"""
  Logger for ZeroMQ communication
"""

from threading import Thread

import zmq

from osgar.bus import BusShutdownException


class LogZeroMQ:
    def __init__(self, config, bus):
        self.thread = Thread(target=self.run, daemon=True)

        context = zmq.Context()
        print("Connecting to hello world server ...")
        self.socket = context.socket(zmq.PULL)  # config?
        self.socket.connect("tcp://localhost:5555")  # TODO config

        self.bus = bus

    def start(self):
        self.thread.start()

    def join(self, timeout=None):
        self.thread.join(timeout=timeout)

    def run(self):
        while self.bus.is_alive():
            message = self.socket.recv()
            self.bus.publish('raw', message)

    def request_stop(self):
        self.bus.shutdown()

# vim: expandtab sw=4 ts=4
