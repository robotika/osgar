"""
  Wrapper for PeakCAN USB device
"""
#
# pip install python-can
# https://python-can.readthedocs.io/en/2.1.0/interfaces/pcan.html
#
# https://www.peak-system.com/
# https://www.peak-system.com/fileadmin/media/files/pcan-basic.zip
# http://www.peak-system.com/quick/BasicLinux
#
# https://en.wikipedia.org/wiki/CAN_bus
#
# The new CAN messages will contain arbitration_id, data and flags
#  http://docs.ros.org/melodic/api/can_msgs/html/msg/Frame.html
#

import struct

import can
from threading import Thread

from osgar.bus import BusShutdownException

IS_EXTENDED_ID_MASK = 0x1

class PeakCAN:
    def __init__(self, config, bus):
        self.bus = bus
        self.canbus = can.interface.Bus(bustype='pcan', channel='PCAN_USBBUS1', bitrate=500000)
        self.input_thread = Thread(target=self.run_input, daemon=True)
        self.output_thread = Thread(target=self.run_output, daemon=True)

    def start(self):
        self.input_thread.start()
        self.output_thread.start()

    def join(self, timeout=None):
        self.input_thread.join(timeout=timeout)
        self.output_thread.join(timeout=timeout)

    def run_input(self):
        while self.bus.is_alive():
            msg = self.canbus.recv()  # TODO timeout
            flags = IS_EXTENDED_ID_MASK if msg.is_extended_id else 0
            self.bus.publish('can', [msg.arbitration_id, msg.data, flags])

    def slot_raw(self, timestamp, packet):
        arbitration_id, data, flags = packet
        msg = can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=(flags & IS_EXTENDED_ID_MASK))
        self.canbus.send(msg)  # TODO timeout, locks?!

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
