"""
  RasPI SPI communication
"""

# https://www.raspberrypi.org/documentation/hardware/raspberrypi/spi/README.md
# https://pypi.org/project/spidev/

# https://www.raspberrypi.org/documentation/usage/gpio/python/README.md

import math

import spidev
from gpiozero import LED, Button

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
        self.spi = spidev.SpiDev()
        self.spi.max_speed_hz = SPI_CLOCK_SPEED
        self.fd = self.spi.open('/dev/spidev0')

        self.pin_request = LED(DATA_REQUEST)
        self.pin_ready = Button(DATA_READY)
#    wiringPiSetup();
#    digitalWrite(DATA_REQUEST, 1);
#    pinMode(DATA_REQUEST, OUTPUT);
#    pinMode(DATA_READY, INPUT);

    def on_raw(self, data):
        # TODO send data via SPI

        self.publish('raw', bytes([0] * 32))

    def update(self):
        channel = super().update()  # define self.time
        handler = getattr(self, "on_" + channel, None)
        if handler is not None:
            handler(getattr(self, channel))
        else:
            assert False, channel  # not supported
        return channel


# vim: expandtab sw=4 ts=4
