"""
  Driver for robo-boat Marina 2.0
"""
from threading import Thread
import struct
import math

from osgar.bus import BusShutdownException


def word_arr(val):
    """return two elements list with LSB, MSB"""
    return [val & 0xFF, (val >> 8) & 0xFF]


class Marina(Thread):
    def __init__(self, config, bus):
        Thread.__init__(self)
        self.setDaemon(True)
        self.bus = bus

        self.i2c_setup = [
                [0x1E, 'W', 0x1, [0x40]],  # compass, gain 820
            ]

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
            # init i2c modules
            for cmd in self.i2c_setup:
                self.bus.publish('cmd', cmd)

            while True:
                for cmd in self.i2c_loop:
                    self.bus.publish('cmd', cmd)
                i2c_count = 0
                while i2c_count < len(self.i2c_loop):
                    dt, src, data = self.bus.listen()
                    if src == 'i2c':
                        i2c_count += 1
                        assert len(data) == 3, data
                        addr, reg, arr = data
                        if addr == 0x1E:  # compass
                            assert reg == 3, reg
                            assert len(arr) == 6, arr
                            x, z, y = struct.unpack('>hhh', bytes(arr))  # axis Y and Z swapped in orig
#                            print('%d\t%d\t%d'% (x, y, z))
                            cx, cy = 525, -1500  # TODO calibration
                            heading = math.atan2(x - cx, cy - y)
                            self.bus.publish('heading', heading)

                    # TODO recovery in case of i2c failure
                    # TODO wait for last element in loop? how long?
                    if src == 'move':
                        speed, angular_speed = data  # raw RC channels
                        self.bus.publish('cmd',  # CHANNEL_MOVE = 2
                                         [0x08, 'W', 0x02, word_arr(speed)])
                        self.bus.publish('cmd',  # CHANNEL_TURN = 0 
                                         [0x08, 'W', 0x00, word_arr(angular_speed)])
        except BusShutdownException:
            pass

    def request_stop(self):
        self.bus.shutdown()


# vim: expandtab sw=4 ts=4
