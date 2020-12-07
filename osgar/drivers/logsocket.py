"""
  Wrapper & timestamper of input/out over socket
"""

import socket
import urllib.request
from threading import Thread

from osgar.logger import LogWriter
from osgar.bus import BusShutdownException


class LogSocket:
    def __init__(self, socket, config, bus):
        bus.register('raw')
        self.verbose = False
        self.socket = socket

        self.input_thread = Thread(target=self.run_input, daemon=True)
        self.output_thread = Thread(target=self.run_output, daemon=True)

        host = config.get('host')
        port = config.get('port')
        self.pair = (host, port)  # (None, None) for unknown address
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
                __, channel, data = self.bus.listen()
                if channel == 'raw':
                    self._send(data)
                else:
                    assert False, channel  # unsupported channel
 
        except BusShutdownException:
            pass

    def request_stop(self):
        self.bus.shutdown()


class LogTCPBase(LogSocket):
    """
      TCP base class for different use cases
    """
    def __init__(self, config, bus):
        soc = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # https://stackoverflow.com/questions/31826762/python-socket-send-immediately
        # https://stackoverflow.com/questions/3761276/when-should-i-use-tcp-nodelay-and-when-tcp-cork
        soc.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        super().__init__(soc, config, bus)

    def _send(self, data):
        self.socket.send(data)


class LogTCPStaticIP(LogTCPBase):
    """
      TCP driver for existing static IP (i.e. SICK LIDAR)
    """
    def __init__(self, config, bus):
        super().__init__(config, bus)
        try:
            self.socket.connect(self.pair)
        except socket.timeout as e:
            print('Timeout', self.pair)
            raise


class LogTCPDynamicIP(LogTCPBase):
    """
      TCP driver for dynamic previously unknown address
         (ROS proxy for subscribers)
    """
    def start(self):
        # on start the address is unknown - it will be received by "output_thread"
        self.output_thread.start()

    def join(self, timeout=None):
        # the "input_thread" is triggered by "output_thread" so make sure
        # that "output_thread" is finished and cannot cause race condition
        self.output_thread.join(timeout=timeout)
        try:
            self.input_thread.join(timeout=timeout)
        except RuntimeError:
            pass

    def run_output(self):
        try:
            while True:
                __, channel, data = self.bus.listen()
                if channel == 'raw':
                    self._send(data)
                elif channel == 'addr':
                    self.pair = tuple(data)
                    self.socket.connect(self.pair)
                    if not self.input_thread.is_alive():
                        self.input_thread.start()
                else:
                    assert False, channel  # unsupported channel
 
        except BusShutdownException:
            pass


class LogTCPServer(LogTCPBase):
    """
      TCP driver for server side - prepare connection and wait
      for others to connect (ROS proxy for publishers)
    """

    def __init__(self, config, bus):
        super().__init__(config, bus)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind(self.pair)
        self.timeout = config.get('timeout')

    def run_input(self):
        if self.verbose:
            print("Waiting ...")
        self.socket.listen(1)
        if self.verbose:
            print("end of listen")
        while self.bus.is_alive():
            try:
                self.socket, addr = self.socket.accept()
                if self.verbose:
                    print('Connected by', addr)
                if self.timeout is not None:
                    self.socket.settimeout(self.timeout)
                super().run_input()
                break
            except socket.timeout:
                pass


class LogUDP(LogSocket):
    def __init__(self, config, bus):
        soc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        LogSocket.__init__(self, soc, config, bus)
        try:
            self.socket.bind(('', self.pair[1]))
        except OSError as e:
            if e.errno == 98: # [Errno 98] Address already in use
                self.socket.connect(('', self.pair[1]))

    def _send(self, data):
        self.socket.sendto(data, self.pair)


class LogHTTP:
    def __init__(self, config, bus):
        bus.register('raw')
        self.input_thread = Thread(target=self.run_input, daemon=True)

        self.url = config['url']
        self.sleep = config.get('sleep', None)
        self.bus = bus

    def start(self):
        self.input_thread.start()

    def join(self, timeout=None):
        self.input_thread.join(timeout=timeout)

    def run_input(self):
        while self.bus.is_alive():
            try:
                # https://github.com/mesonbuild/meson/issues/4087
                # without timeout the call can hang the process forever
                with urllib.request.urlopen(self.url, timeout=0.5) as f:
                    data = f.read()
                if len(data) > 0:
                    self.bus.publish('raw', data)
            except socket.timeout:
                pass
            if self.sleep is not None:
                self.bus.sleep(self.sleep)

    def request_stop(self):
        self.bus.shutdown()


if __name__ == "__main__":
    import time
    from osgar.bus import Bus

    config = {'host':'localhost', 'port': 8001, 'timeout': 1.0}
    with LogWriter(prefix='test-tcp-') as log:
        bus = Bus(log)
        device = LogTCPStaticIP(config, bus=bus.handle('tcp'))
        device.start()
        time.sleep(2)
        device.request_stop()
        device.join()

# vim: expandtab sw=4 ts=4
