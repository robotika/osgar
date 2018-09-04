"""
  Driver for robo-boat Marina 2.0
"""
from threading import Thread

from osgar.bus import BusShutdownException


class Marina(Thread):
    def __init__(self, config, bus):
        Thread.__init__(self)
        self.setDaemon(True)
        self.bus = bus

        self.i2c_loop = [
                [0x1E, 'R', 0x03, 6],  # compass HMC5883L
                [0x68, 'R', 0x1B, 8],  # gyro ITG-3205
                [0x53, 'R', 0x32, 6],  # acc ADXL345
                [0x08, 'R', 0x00, 2],  # Arduino - channel 0
                [0x08, 'R', 0x01, 2],
                [0x08, 'R', 0x02, 2],
                [0x08, 'R', 0x03, 2],
                [0x08, 'R', 0x04, 2],
                [0x08, 'R', 0x05, 2],
                [0x08, 'R', 0x80, 1],  # Arduino A/M state
            ]

    def run(self):
        try:
            # TODO init i2c modules
            while True:
                for cmd in self.i2c_loop:
                    self.bus.publish('cmd', cmd)
                for __ in self.i2c_loop:
                    dt, src, data = self.bus.listen()
                    # TODO recovery in case of i2c failure
        except BusShutdownException:
            pass

    def request_stop(self):
        self.bus.shutdown()


# vim: expandtab sw=4 ts=4
