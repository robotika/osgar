"""
  Skiddy workaround with more regular updates of serial line
"""

import serial
from threading import Thread

from osgar.logger import LogWriter
from osgar.bus import BusShutdownException


class SerialLoop:
    def __init__(self, config, bus):
        bus.register('raw')
        self.input_thread = Thread(target=self.run_input, daemon=True)
        self.output_thread = Thread(target=self.run_output, daemon=True)

        if 'port' in config:
            if config.get('rtscts'):
                self.com = serial.Serial(config['port'], config['speed'], rtscts=True)
                self.com.setRTS()
            else:
                self.com = serial.Serial(config['port'], config['speed'])
            self.com.timeout = config.get('timeout', 0.02)  # default expects updates ~ 50Hz
            if config.get('reset'):
                self.com.setDTR(0)
        else:
            self.com = None
        self.bus = bus

        self.init_raw_data = b'\x00\x00\x03\x01\x01\xfb'  # query version
        self.loop_raw_data = b'\x00\x00\x0f\x01\x0e\x00\x00\x00\x00\x00\x00\x00\x80@\x05\x00\x00\x1d'  # speed 0, 0


    def start(self):
        self.input_thread.start()
        self.output_thread.start()

    def join(self, timeout=None):
        self.input_thread.join(timeout=timeout)
        self.output_thread.join(timeout=timeout)

    def run_input(self):
        self.com.write(self.init_raw_data)
        while self.bus.is_alive():
            data = self.com.read(1024)
            if len(data) > 0:
                self.com.write(self.loop_raw_data)
                self.bus.publish('raw', data)

    def slot_raw(self, timestamp, data):
        self.loop_raw_data = data

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
