"""
  Driver for a R-Team robot Maxi 2021

  The main computer is RasPI and communication is via SPI.
"""

# https://www.raspberrypi.org/documentation/hardware/raspberrypi/spi/README.md
# https://pypi.org/project/spidev/


import math

import spidev

from osgar.node import Node
from osgar.bus import BusShutdownException


SPI_CHANNEL = 0
SPI_CLOCK_SPEED = 16000000


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


class Maxi2021(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('pose2d', 'emergency_stop', 'encoders', 'raw')

        # commands
        self.desired_speed = 0.0  # m/s
        self.desired_angular_speed = 0.0

        # status
        self.emergency_stop = None  # uknown state
        self.pose2d = (0.0, 0.0, 0.0)  # x, y in meters, heading in radians (not corrected to 2PI)
        self.buttons = None

        self.verbose = False  # should be in Node

        # Enable SPI
        self.spi = spidev.SpiDev()
        sellf.spi.max_speed_hz = SPI_CLOCK_SPEED

    def send_pose2d(self):
        x, y, heading = self.pose2d
        self.publish('pose2d', [round(x*1000), round(y*1000),
                                round(math.degrees(heading)*100)])

    def update(self):
        channel = super().update()  # define self.time
        handler = getattr(self, "on_" + channel, None)
        if handler is not None:
            handler(getattr(self, channel))
        else:
            assert False, channel  # not supported
        return channel


# vim: expandtab sw=4 ts=4
