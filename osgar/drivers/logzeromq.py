"""
  Logger for ZeroMQ communication
"""

from threading import Thread

import zmq

from osgar.bus import BusShutdownException


class LogZeroMQ:
    def __init__(self, config, bus):
        bus.register('raw:gz' if config.get('save_data', False) else 'raw:null', 'response')
        mode = config['mode']
        self.endpoint = config['endpoint']
        self.timeout = config.get('timeout', 1) # default recv timeout 1s

        if mode == 'PULL':
            self.thread = Thread(target=self.run_input)
        elif mode == 'PUSH':
            self.thread = Thread(target=self.run_output)
        elif mode == 'REQ':
            self.thread = Thread(target=self.run_reqrep)
        else:
            assert False, mode  # unknown/unsupported mode

        self.thread.name = bus.name
        self.bus = bus

    def start(self):
        self.thread.start()

    def join(self, timeout=None):
        self.thread.join(timeout=timeout)

    def run_input(self):
        context = zmq.Context()
        socket = context.socket(zmq.PULL)
        # https://stackoverflow.com/questions/7538988/zeromq-how-to-prevent-infinite-wait
        socket.RCVTIMEO = int(self.timeout * 1000)  # convert to milliseconds
        socket.connect(self.endpoint)

        while self.bus.is_alive():
            try:
                message = socket.recv()
                self.bus.publish('raw', message)
            except zmq.error.Again:
                pass
        socket.close()
        context.term()

    def run_output(self):
        context = zmq.Context()
        socket = context.socket(zmq.PUSH)
        # https://stackoverflow.com/questions/24619490/how-do-i-clear-the-buffer-upon-start-exit-in-zmq-socket-to-prevent-server-from
        # ZMQ_LINGER: Set linger period for socket shutdown
        # The default value of -1 specifies an infinite linger period.
        # Pending messages shall not be discarded after a call to
        # zmq_close(); attempting to terminate the socket's context with
        # zmq_term() shall block until all pending messages have been sent
        # to a peer.
        socket.setsockopt(zmq.LINGER, 100)  # milliseconds
        socket.connect(self.endpoint)
        try:
            while True:
                dt, __, data = self.bus.listen()
                while self.bus.is_alive():
                    try:
                        socket.send(data, zmq.NOBLOCK)
                        break
                    except zmq.error.Again:
                        self.bus.sleep(0.1)
        except BusShutdownException:
            pass
        socket.close()
        context.term()

    def run_reqrep(self):
        context = zmq.Context()
        socket = context.socket(zmq.REQ)
        try:
            while True:
                dt, __, data = self.bus.listen()
                socket.send_string(data)
                self.bus.publish('response', socket.recv())
        except BusShutdownException:
            pass
        socket.close()
        context.term()

    def request_stop(self):
        self.bus.shutdown()

# vim: expandtab sw=4 ts=4
