"""
  John Deere Driver
"""


import ctypes
import serial
import struct
from threading import Thread

from osgar.logger import LogWriter, LogReader
from osgar.bus import BusShutdownException


CAN_ID_ENCODERS = 0x284


def sint16_diff(a, b):
    '''clip integer value (a - b) to signed int16'''
    assert 0 <= a <= 0xFFFF, a
    assert 0 <= b <= 0xFFFF, b
    return ctypes.c_short(a - b).value


class JohnDeere(Thread):
    def __init__(self, config, bus):
        Thread.__init__(self)
        self.setDaemon(True)

        self.bus = bus

        self.desired_speed = 0.0  # m/s
        self.desired_steering = None  # not specified, no need to change

        self.prev_enc_raw = None
        self.dist_left_raw = 0
        self.dist_right_raw = 0

    def update_encoders(self, data):
        print('ENC', data)
        assert len(data) == 4, data
        arr = [data[2*i+1]*256 + data[2*i] for i in range(2)]
        if self.prev_enc_raw is not None:
            diffL = sint16_diff(arr[0], self.prev_enc_raw[0])
            diffR = sint16_diff(arr[1], self.prev_enc_raw[1])

            if abs(diffL) > 128:
                print("ERR-L\t{}\t{}\t{}".format(self.dist_left_raw, self.prev_enc_raw[0], arr[0]))
            else:
                self.dist_left_raw += diffL

            if abs(diffR) > 128:
                print("ERR-R\t{}\t{}\t{}".format(self.dist_right_raw, self.prev_enc_raw[1], arr[1]))
            else:
                self.dist_right_raw += diffR
        self.prev_enc_raw = arr

    def process_packet(self, packet, verbose=False):
        if len(packet) >= 2:
            msg_id = ((packet[0]) << 3) | (((packet[1]) >> 5) & 0x1f)
            print(hex(msg_id), packet[2:])
            if msg_id == CAN_ID_ENCODERS:
                self.update_encoders(packet[2:])
                self.bus.publish('encoders', [self.dist_left_raw,  self.dist_right_raw])

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
                    self.desired_speed = data
                elif channel == 'desired_steering':
                    self.desired_steering = data
                else:
                    assert False, channel  # unsupported channel
        except BusShutdownException:
            pass

    def request_stop(self):
        self.bus.shutdown()

# vim: expandtab sw=4 ts=4
