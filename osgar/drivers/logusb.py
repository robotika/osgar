"""
  Wrapper for USB communication
"""

from threading import Thread
import time

# pyusb
import usb.core
import usb.util

from osgar.bus import BusShutdownException


class LogUSB:
    def __init__(self, config, bus):
        self.input_thread = Thread(target=self.run_input, daemon=True)
        self.output_thread = Thread(target=self.run_output, daemon=True)

        # TODO make configurable for other devices then SICK lidar TIM310
        self.dev = usb.core.find(idVendor=0x19A2, idProduct=0x5001)
        for i in range(10):
            try:
                self.dev.set_configuration()
                break
            except:
                print("LaserUSB - init ERROR", i)
                time.sleep(0.1)

        self.bus = bus

    def start(self):
        self.input_thread.start()
        self.output_thread.start()

    def join(self, timeout=None):
        self.input_thread.join(timeout=timeout)
        self.output_thread.join(timeout=timeout)

    def run_input(self):
        while self.bus.is_alive():
            data = self.dev.read(1|usb.ENDPOINT_IN, 65535, timeout = 100)
            if len(data) > 0:
                self.bus.publish('raw', bytes(data))

    def run_output(self):
        try:
            while True:
                __, __, data = self.bus.listen()
                self.dev.write(2|usb.ENDPOINT_OUT, data, 0)
        except BusShutdownException:
            pass

    def request_stop(self):
        self.bus.shutdown()

# vim: expandtab sw=4 ts=4
