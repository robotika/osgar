"""
  Wrapper & timestamper of input/out over socket
"""

import socket
import urllib.request
import time
from threading import Thread

from osgar.logger import LogWriter
from osgar.bus import BusShutdownException


class LogSocket:
    def __init__(self, socket, config, bus):
        self.socket = socket

        self.input_thread = Thread(target=self.run_input, daemon=True)
        self.output_thread = Thread(target=self.run_output, daemon=True)

        host = config['host']
        port = config['port']
        self.pair = (host, port)
        if 'timeout' in config:
            self.socket.settimeout(config['timeout'])
        self.bufsize = config.get('bufsize', 1024)

        self.bus = bus

    def _send(self, data):
        raise NotImplementedError()

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
                self._send(data)
        except BusShutdownException:
            pass

    def request_stop(self):
        self.bus.shutdown()


class LogTCP(LogSocket):
    def __init__(self, config, bus):
        soc = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        LogSocket.__init__(self, soc, config, bus)
        self.socket.connect(self.pair)

    def _send(self, data):
        self.socket.send(data)


class LogUDP(LogSocket):
    def __init__(self, config, bus):
        soc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        LogSocket.__init__(self, soc, config, bus)
        self.socket.bind(('', self.pair[1]))

    def _send(self, data):
        self.socket.sendto(data, self.pair)


class LogHTTP:
    def __init__(self, config, bus):
        self.input_thread = Thread(target=self.run_input, daemon=True)

        self.url = config['url']
        self.sleep = config.get('sleep')
        if 'timeout' in config:
            socket.setdefaulttimeout(config['timeout'])
        self.bus = bus

    def start(self):
        self.input_thread.start()

    def join(self, timeout=None):
        self.input_thread.join(timeout=timeout)

    def run_input(self):
        while self.bus.is_alive():
            try:
                with urllib.request.urlopen(self.url) as f:
                    data = f.read()
                if len(data) > 0:
                    self.bus.publish('raw', data)
                    if self.sleep is not None:
                        time.sleep(self.sleep)  # TODO skip in replay
            except urllib.error.URLError as e:
                self.bus.report_error(e)

    def request_stop(self):
        self.bus.shutdown()


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
