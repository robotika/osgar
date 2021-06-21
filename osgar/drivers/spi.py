"""
  RasPI SPI communication
"""

# https://www.raspberrypi.org/documentation/hardware/raspberrypi/spi/README.md
# https://pypi.org/project/spidev/

# https://www.raspberrypi.org/documentation/usage/gpio/python/README.md

import math

import spidev
import RPi.GPIO as GPIO

from osgar.node import Node
from osgar.bus import BusShutdownException


SPI_CHANNEL = 0
SPI_CLOCK_SPEED = 16000000

DATA_REQUEST = 9
DATA_READY = 8


# output command structure
"""
typedef union {
  struct __attribute__((__packed__)) {
    uint8_t Command;
    int16_t Speed;
    int16_t Ride;
  };
  uint8_t data[8];
} TDrive;
"""

# input status structure
"""
typedef union {
  struct __attribute__((__packed__)) {
    uint32_t  Time;  // ms since start 
    uint8_t   Status;  // bit status, power, batteries
    int32_t   Position;  // in mm
    int16_t   Velocity;  // in mm/s
    uint16_t  Voltage;  // fractional 8bit whole and 8bit fraction
    uint16_t  Current;  // fractional 8bit whole and 8bit fraction
    int16_t   Roll, Pitch, Yaw;  // 1/100th deg

    uint16_t  SonarSide;  // rotating sonar - dist in mm
    uint16_t  SonarFrontLeft;
    uint16_t  SonarFrontRight;
  };
  uint8_t data[32];
} TScan;
"""


class Spi(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('raw')

        self.verbose = False  # should be in Node

        # Enable SPI
        self.spi = self.init()

    def init(self):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)  # Cislovani pinu - BCM nebo BOARD
        GPIO.setup(DATA_REQUEST, GPIO.OUT)  # Nastaveni pinu jako vystupu
        GPIO.setup(DATA_READY, GPIO.IN)  # Nastaveni pinu jako vstupu

        spi = spidev.SpiDev()
        spi.open(0, 0)  # corresponds to '/dev/spidev0.0'
        spi.max_speed_hz = 16000000
        return spi

    def send_command(self, command, speed, ride):
        buffer = [command]
        buffer = buffer + [speed % 256, speed // 256]
        buffer = buffer + [ride % 256, ride // 256]
        buffer = buffer + [0x00, 0x00, 0x00]
        self.spi.writebytes(buffer)

    def read_data(self):
        GPIO.output(DATA_REQUEST, GPIO.LOW)
        while GPIO.input(DATA_READY) == 1:
            print('.')

        buffer = self.spi.readbytes(32)

        GPIO.output(DATA_REQUEST, GPIO.HIGH)
        while GPIO.input(DATA_READY) == 0:
            print('o')

        return buffer

    def on_raw(self, data):
        self.spi.writebytes(data)
        buf = self.read_data()
        self.publish('raw', bytes(buf))

    def update(self):
        channel = super().update()  # define self.time
        handler = getattr(self, "on_" + channel, None)
        if handler is not None:
            handler(getattr(self, channel))
        else:
            assert False, channel  # not supported
        return channel


# vim: expandtab sw=4 ts=4
