"""
  Logger for ZeroMQ communication
"""

from threading import Thread

import zmq

from osgar.bus import BusShutdownException


class LogZeroMQ:
    def __init__(self, config, bus):
        mode = config['mode']
        endpoint = config['endpoint']

        context = zmq.Context()
        print("Connecting to hello world server ...")
        if mode == 'PULL':
            self.socket = context.socket(zmq.PULL)
            self.thread = Thread(target=self.run_input, daemon=True)
        elif mode == 'PUSH':
            self.socket = context.socket(zmq.PUSH)
            self.thread = Thread(target=self.run_output, daemon=True)
        else:
            assert False, mode  # unknown/unsupported mode

        self.socket.connect(endpoint)  # TODO config

        self.bus = bus

    def start(self):
        self.thread.start()

    def join(self, timeout=None):
        self.thread.join(timeout=timeout)

    def run_input(self):
        while self.bus.is_alive():
            message = self.socket.recv()
            self.bus.publish('raw', message)

    def slot_raw(self, timestamp, data):
        self.socket.send(data)

    def run_output(self):
        try:
            while True:
                dt, __, data = self.bus.listen()
                self.slot_raw(dt, data)
        except BusShutdownException:
            pass

    def request_stop(self):
        self.bus.shutdown()

# vim: expandtab sw=4 ts=4
