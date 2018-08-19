"""
  Wrapper & timestamper of input/out over socket
"""

import socket
from threading import Thread

from osgar.logger import LogWriter
from osgar.bus import BusShutdownException


class LogTCP:
    def __init__(self, config, bus):
        self.input_thread = Thread(target=self.run_input, daemon=True)
        self.output_thread = Thread(target=self.run_output, daemon=True)

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        host = config['host']
        port = config['port']
        self.socket.connect((host, port))
        if 'timeout' in config:
            self.socket.settimeout(config['timeout'])
        self.bufsize = config.get('bufsize', 1024)

        self.bus = bus

    def start(self):
        self.input_thread.start()
        self.output_thread.start()

    def join(self, timeout=None):
        self.input_thread.join(timeout=timeout)
        self.output_thread.join(timeout=timeout)

    def run_input(self):
        while self.bus.is_alive():
            try:
                data = self.socket.recv(self.bufsize)
                if len(data) > 0:
                    self.bus.publish('raw', data)
            except socket.timeout:
                pass

    def run_output(self):
        try:
            while True:
                __, __, data = self.bus.listen()
                self.socket.send(data)
        except BusShutdownException:
            pass

    def request_stop(self):
        self.bus.shutdown()


class LogUDP(LogTCP):
    pass


if __name__ == "__main__":
    import time
    from osgar.bus import BusHandler

    config = {'host':'localhost', 'port': 8001, 'timeout': 1.0}
    log = LogWriter(prefix='test-tcp-')
    device = LogTCP(config, bus=BusHandler(log, out={'raw':[]}))
    device.start()
    time.sleep(2)
    device.request_stop()
    device.join()

# vim: expandtab sw=4 ts=4
