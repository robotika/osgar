"""
  Driver for robot Eduro
"""


import ctypes
import serial
import struct
from threading import Thread

from osgar.logger import LogWriter, LogReader
from osgar.bus import BusShutdownException
from .canserial import CAN_packet

CAN_ID_SYNC = 0x80
CAN_ID_ENCODERS_LEFT = 0x181
CAN_ID_ENCODERS_RIGHT = 0x182
CAM_SYSTEM_STATUS = 0x8A  # including emergency STOP button


def sint16_diff(a, b):
    '''clip integer value (a - b) to signed int16'''
    assert 0 <= a <= 0xFFFF, a
    assert 0 <= b <= 0xFFFF, b
    return ctypes.c_short(a - b).value


class Eduro(Thread):
    def __init__(self, config, bus):
        Thread.__init__(self)
        self.setDaemon(True)

        self.bus = bus

        self.desired_speed = 0.0  # m/s
        self.desired_angular_speed = 0.0

        self.prev_enc_raw = {}
        self.dist_left_raw = 0
        self.dist_right_raw = 0

        self.emergency_stop = None  # uknown state

    def update_encoders(self, msg_id, data):
#        print('ENC', hex(msg_id), data)
        assert len(data) == 4, data
        arr = struct.unpack('<i', data)
        if msg_id in self.prev_enc_raw:
#            diff = sint16_diff(arr[0], self.prev_enc_raw[msg_id])
            diff = arr[0] - self.prev_enc_raw[msg_id]

            if msg_id == CAN_ID_ENCODERS_LEFT:
                self.dist_left_raw += diff
            elif msg_id == CAN_ID_ENCODERS_RIGHT:
                self.dist_right_raw += diff
        self.prev_enc_raw[msg_id] = arr[0]

    def update_emergency_stop(self, msg_id, data):
        assert len(data) == 8, len(data)
        self.emergency_stop = (data[:2] == bytes([0,0x10])) and (data [3:] == bytes([0,0,0,0,0]))

    def send_speed(self):
        if self.desired_speed > 0:
            left, right = 1000, 1000
        else:
            left, right = 0, 0
        self.bus.publish('can', CAN_packet(0x201, [
            left&0xff, (left>>8)&0xff,
            right&0xff, (right>>8)&0xff]))

    def process_packet(self, packet, verbose=False):
        if len(packet) >= 2:
            msg_id = ((packet[0]) << 3) | (((packet[1]) >> 5) & 0x1f)
#            print(hex(msg_id), packet[2:])
            if msg_id in [CAN_ID_ENCODERS_LEFT, CAN_ID_ENCODERS_RIGHT]:
                self.update_encoders(msg_id, packet[2:])
            elif msg_id == CAM_SYSTEM_STATUS:
                self.update_emergency_stop(msg_id, packet[2:])
                self.bus.publish('emergency_stop', self.emergency_stop)
            elif msg_id == CAN_ID_SYNC:
                self.bus.publish('encoders', [self.dist_left_raw,  self.dist_right_raw])
                self.send_speed()

    def process_gen(self, data, verbose=False):
        self.process_packet(data)
        yield None

    def run(self):
        try:
            while True:
                dt, channel, data = self.bus.listen()
                if channel == 'can':
                    if len(data) > 0:
                        for status in self.process_gen(data):
                            if status is not None:
                                self.bus.publish('can', status)  # TODO
                elif channel == 'desired_speed':
                    self.desired_speed, self.desired_angular_speed = data
                else:
                    assert False, channel  # unsupported channel
        except BusShutdownException:
            pass

    def request_stop(self):
        self.bus.shutdown()

# vim: expandtab sw=4 ts=4
