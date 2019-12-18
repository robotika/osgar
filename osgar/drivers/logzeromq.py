"""
  Logger for ZeroMQ communication
"""

from threading import Thread

import zmq

from osgar.bus import BusShutdownException


class LogZeroMQ:
    def __init__(self, config, bus):
        bus.register('raw:null')
        mode = config['mode']
        endpoint = config['endpoint']

        self.context = zmq.Context()
        print("Connecting to hello world server ...")
        if mode == 'PULL':
            self.socket = self.context.socket(zmq.PULL)
            if 'timeout' in config:
                # https://stackoverflow.com/questions/7538988/zeromq-how-to-prevent-infinite-wait
                self.socket.RCVTIMEO = int(config['timeout'] * 1000) # in milliseconds
            self.thread = Thread(target=self.run_input, daemon=False)
        elif mode == 'PUSH':
            self.socket = self.context.socket(zmq.PUSH)
            # https://stackoverflow.com/questions/24619490/how-do-i-clear-the-buffer-upon-start-exit-in-zmq-socket-to-prevent-server-from
            # ZMQ_LINGER: Set linger period for socket shutdown
            # The default value of -1 specifies an infinite linger period.
            # Pending messages shall not be discarded after a call to
            # zmq_close(); attempting to terminate the socket's context with
            # zmq_term() shall block until all pending messages have been sent
            # to a peer.
            self.socket.setsockopt(zmq.LINGER, 100)  # milliseconds
            self.thread = Thread(target=self.run_output, daemon=False)
        else:
            assert False, mode  # unknown/unsupported mode

        self.socket.connect(endpoint)

        self.bus = bus

    def start(self):
        self.thread.start()

    def join(self, timeout=None):
        self.thread.join(timeout=timeout)

    def _close(self):
        self.socket.close()
        self.context.term()

    def run_input(self):
        while self.bus.is_alive():
            try:
                message = self.socket.recv()
                self.bus.publish('raw', message)
            except zmq.error.Again:
                pass
        self._close()

    def slot_raw(self, timestamp, data):
        while self.bus.is_alive():
            try:
                self.socket.send(data, zmq.NOBLOCK)
                break
            except zmq.error.Again:
                self.bus.sleep(0.1)

    def run_output(self):
        try:
            while True:
                dt, __, data = self.bus.listen()
                self.slot_raw(dt, data)
        except BusShutdownException:
            pass
        self._close()

    def request_stop(self):
        self.bus.shutdown()

# vim: expandtab sw=4 ts=4
