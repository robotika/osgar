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
from threading import Thread, Event

from lib.logger import LogWriter
from drivers.bus import BusShutdownException


class LogSerial(Thread):
    def __init__(self, config, bus, com=None):
        Thread.__init__(self)
        self.setDaemon(True)
        self.should_run = Event()
        self.should_run.set()

        if com is None:
            if 'port' in config:
                self.com = serial.Serial(config['port'], config['speed'])
                self.com.timeout = 0.01  # expects updates < 100Hz
            else:
                self.com = None
        else:
            self.com = com
        self.bus = bus

        self.buf = b''

    def run(self):
        while self.should_run.isSet():
            data = self.com.read(1024)
            if len(data) > 0:
                self.bus.publish('raw', data)

    def request_stop(self):
        self.should_run.clear()


class LogSerialOut(Thread):
    def __init__(self, config, bus, com=None):
        Thread.__init__(self)
        self.setDaemon(True)

        if com is None:
            if 'port' in config:
                self.com = serial.Serial(config['port'], config['speed'])
                self.com.timeout = 0.01  # expects updates < 100Hz
            else:
                self.com = None
        else:
            self.com = com
        self.bus = bus

    def run(self):
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
