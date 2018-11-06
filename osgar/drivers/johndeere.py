"""
  John Deere Driver
"""


import ctypes
import serial
import struct
from threading import Thread

from osgar.logger import LogWriter, LogReader
from osgar.bus import BusShutdownException
from .canserial import CAN_packet


CAN_ID_ENCODERS = 0x284
CAN_ID_WHEEL_ANGLE_STATUS = 0x182


# accepted +/- for completed turn
TURN_TOLERANCE = 10
TURN_TOLERANCE_INTEGRAL = 100

# number of samples for left and right encoders for average speed computation and control
SPEED_PERIOD = 20
SPEED_TOLERANCE = 10

CENTER_GAS_MIN = -3768
CENTER_GAS_MAX = 232
GO_LIMIT = 6000  # go-go action?
STOP_CENTER = (CENTER_GAS_MIN + CENTER_GAS_MAX)/2
GO_BACK_LIMIT = -9000

MAX_GAS_LIMIT = 9000
MIN_GAS_LIMIT = -12000
GAS_STEP = 500

# meters per single encoder tick
ENC_SCALE = 2*3.39/float(252 + 257)


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

        self.desired_speed_raw = None  # if not None then regulate speed
        self.desired_wheel_angle_raw = None
        self.wheel_angle_raw = None
        self.desired_wheel_tiny_corrections = None

        self.prev_enc_raw = None
        self.dist_left_diff = 0
        self.dist_right_diff = 0
        self.dist_period_sum = 0  # compute average for both encoders
        self.dist_count = 0
        self.valid_speed_ref = False  # was speed measured for the whole period

        self.pedal_position = None
        self.cmd = None  # single shot command (can be issued by speed control)
        self.last_sent_speed_cmd = None

    def update_encoders(self, data):
        assert len(data) == 4, data
        arr = [data[2*i+1]*256 + data[2*i] for i in range(2)]
        if self.prev_enc_raw is not None:
            diffL = sint16_diff(arr[0], self.prev_enc_raw[0])
            diffR = sint16_diff(arr[1], self.prev_enc_raw[1])

            if abs(diffL) > 128:
                print("ERR-L\t{}\t{}\t{}".format(self.dist_left_diff, self.prev_enc_raw[0], arr[0]))
            else:
                self.dist_left_diff = diffL
                self.dist_period_sum += diffL
                self.dist_count += 1

            if abs(diffR) > 128:
                print("ERR-R\t{}\t{}\t{}".format(self.dist_right_diff, self.prev_enc_raw[1], arr[1]))
            else:
                self.dist_right_diff = diffR
                self.dist_period_sum += diffR
                self.dist_count += 1
        self.prev_enc_raw = arr

    def update_wheel_angle_status(self, data):
        assert(len(data) == 2) 
        self.wheel_angle_raw = ctypes.c_short(data[1]*256 + data[0]).value
        if self.desired_wheel_tiny_corrections is not None:
            self.wheel_angle_raw_integral += self.wheel_angle_raw - self.desired_wheel_tiny_corrections

    def send_can_data(self, msg_id, data):
        self.publish('can', CAN_packet(msg_id, data))

    def send_steering_cmd(self):
        if self.desired_wheel_angle_raw is not None and self.wheel_angle_raw is not None:
            if abs(self.desired_wheel_angle_raw - self.wheel_angle_raw) > TURN_TOLERANCE:
                self.desired_wheel_tiny_corrections = None
                if self.desired_wheel_angle_raw > self.wheel_angle_raw:
                    self.send_can_data(0x202, [0x5])  # left
                else:
                    self.send_can_data(0x202, [0x6])  # right
            else:
                self.send_can_data(0x202, [0])
                self.desired_wheel_tiny_corrections = self.desired_wheel_angle_raw
                self.wheel_angle_raw_integral = 0
                self.desired_wheel_angle_raw = None

        elif self.desired_wheel_tiny_corrections is not None:
            if abs(self.wheel_angle_raw_integral) > TURN_TOLERANCE_INTEGRAL:
                if self.wheel_angle_raw_integral < 0:
                    self.send_can_data(0x202, [0x5])  # left
                else:
                    self.send_can_data(0x202, [0x6])  # right
                self.wheel_angle_raw_integral = 0  # pulse only
            else:
                self.send_can_data(0x202, [0])

    def send_speed_cmd(self):
        if self.dist_count >= SPEED_PERIOD:
            speed = self.dist_period_sum / self.dist_count
            self.dist_count = 0
            self.dist_period_sum = 0
            if self.desired_speed_raw is not None and self.valid_speed_ref:
                cmd = None
                if speed + SPEED_TOLERANCE < self.desired_speed_raw:
                    cmd = min(self.last_sent_speed_cmd + GAS_STEP, MAX_GAS_LIMIT)
                elif speed - SPEED_TOLERANCE > self.desired_speed_raw:
                    cmd = max(self.last_sent_speed_cmd - GAS_STEP, MIN_GAS_LIMIT)
                if cmd is not None:
                    self.last_sent_speed_cmd = cmd
                    self.send_can_data(0x201, [cmd & 0xFF, (cmd>>8)&0xFF])
                    self.valid_speed_ref = True  # ignore first, but accept following measurements

    def set_desired_speed(self, speed):
        raw_speed = speed / ENC_SCALE  # time?!
        if raw_speed != self.desired_speed_raw:
            # TODO more suitable conversion table for initial command
            if raw_speed is not None:
                if raw_speed == 0:
                    self.cmd = STOP_CENTER
                elif raw_speed > 0:
                    self.cmd = GO_LIMIT
                else:
                    self.cmd = GO_BACK_LIMIT
                self.valid_speed_ref = False
        self.desired_speed_raw = raw_speed

    def process_packet(self, packet, verbose=False):
        if len(packet) >= 2:
            msg_id = (packet[0] << 3) | (packet[1] >> 5)
            if msg_id == CAN_ID_ENCODERS:
                self.update_encoders(packet[2:])
                self.bus.publish('encoders', [self.dist_left_diff,  self.dist_right_diff])
                self.send_speed_cmd()
                self.send_steering_cmd()
            if msg_id == CAN_ID_WHEEL_ANGLE_STATUS:
                self.update_wheel_angle_status(packet[2:])

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
                    self.set_desired_speed(data)

                elif channel == 'desired_steering':
                    self.desired_steering_raw = data  # !!! conversion
                else:
                    assert False, channel  # unsupported channel
        except BusShutdownException:
            pass

    def request_stop(self):
        self.bus.shutdown()

# vim: expandtab sw=4 ts=4
