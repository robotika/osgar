"""
  Wrapper & timestamper of input serial byte stream
"""

### REMOVE THIS WITH PACKAGE OSGAR ###
import sys
import os
import inspect

OSGAR_ROOT = os.path.realpath(os.path.abspath(os.path.join(os.path.split(inspect.getfile( inspect.currentframe() ))[0],"..")))
if OSGAR_ROOT not in sys.path:
    sys.path.insert(0, OSGAR_ROOT) # access to logger without installation
### END OF REMOVAL ###


import serial
from threading import Thread

from lib.logger import LogWriter
from drivers.bus import BusShutdownException


class LogSerial:
    def __init__(self, config, bus):
        self.input_thread = Thread(target=self.run_input, daemon=True)
        self.output_thread = Thread(target=self.run_output, daemon=True)

        if 'port' in config:
            if config.get('rtscts'):
                self.com = serial.Serial(config['port'], config['speed'], rtscts=True)
                self.com.setRTS()
            else:
                self.com = serial.Serial(config['port'], config['speed'])
            self.com.timeout = config.get('timeout', 0.01)  # default expects updates < 100Hz
            if config.get('reset'):
                self.com.setDTR(0)
        else:
            self.com = None
        self.bus = bus

        self.buf = b''

    def start(self):
        self.input_thread.start()
        self.output_thread.start()

    def join(self, timeout=None):
        self.input_thread.join(timeout=timeout)
        self.output_thread.join(timeout=timeout)

    def run_input(self):
        while self.bus.is_alive():
            data = self.com.read(1024)
            if len(data) > 0:
                self.bus.publish('raw', data)

    def run_output(self):
        try:
            while True:
                __, __, data = self.bus.listen()
                self.com.write(data)
        except BusShutdownException:
            pass

    def request_stop(self):
        self.bus.shutdown()


if __name__ == "__main__":
    import time
    from drivers.bus import BusHandler

    config = { 'port': 'COM5', 'speed': 4800 }
    log = LogWriter(prefix='test-')
    device = LogSerial(config, bus=BusHandler(log, out={'raw':[]}))
    device.start()
    time.sleep(2)
    device.request_stop()
    device.join()

# vim: expandtab sw=4 ts=4
