"""
  CAN bus proxy, i.e wrapper for some task which will be handled by CAN later.
  In particular:
      - state machine for gas pedal control
      - timing for turn (without feedback)
"""

import ctypes

PULSE_DURATION = 0.3  #0.5  # seconds

CENTER_GAS_MIN = -3768  # 14500
CENTER_GAS_MAX = 232  # 16500

#GO_LIMIT = 5232  # 19000
GO_LIMIT = 6000  # go-go action?
STOP_CENTER = (CENTER_GAS_MIN + CENTER_GAS_MAX)/2
GO_BACK_LIMIT = -9000

MAX_GAS_LIMIT = 9000
MIN_GAS_LIMIT = -12000
GAS_STEP = 500

SPEED_TOLERANCE = 10
SPEED_UPDATE_CONTROL_FREQ = 2  # Hz

SCALE_NEW = 0.5  # 0.0 < x < 1.0

# accepted +/- for completed turn
TURN_TOLERANCE = 10


def sint16_diff(a, b):
    '''clip integer value (a - b) to signed int16'''
    assert 0 <= a <= 0xFFFF, a
    assert 0 <= b <= 0xFFFF, b
    return ctypes.c_short(a - b).value


class CANProxy:

    def __init__(self, can, verbose=False):
        self.can = can
        self.verbose = verbose
        self.time = 0.0
        self.gas = None  # pedal_position would be more appropriate name
        self.cmd = None  # single shot command (can be issued by speed control)
        self.last_sent_speed_cmd = None

        self.prev_enc_raw = None
        self.dist_left_raw = 0
        self.dist_right_raw = 0
        self.speed_raw = 0
        self.speed_average = 0
        self.speed_arr = [0]*20  # well, this should correspond to 1sec
        self.desired_speed_raw = None  # if not None then regulate speed
        self.valid_speed_ref = False  # was speed measured for the whole period

        self.wheel_angle_raw = None
        self.desired_wheel_angle_raw = None

        self.buttons_and_LEDs = None  # upper nybble contains LED status
        self.cmd_LEDs = None
        self.bumpers = None

    def go(self):
        self.cmd = 'go'
        self.desired_speed_raw = None

    def go_back(self):
        self.cmd = 'go_back'
        self.desired_speed_raw = None

    def go_slowly(self):
        self.cmd = 'go_slowly'
        self.desired_speed_raw = None

    def stop(self):
        self.cmd = 'stop'
        self.desired_speed_raw = None

    def set_desired_speed_raw(self, raw_speed):
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

    def set_turn_raw(self, raw_angle):
        self.desired_wheel_angle_raw = raw_angle

    def stop_turn(self):
        "immediately stop turning = close valves"
        self.can.sendData(0x202, [0])
        self.desired_wheel_angle_raw = None

    def update_gas_status(self, (id, data)):
        if id == 0x181:
            assert len(data)==2, data
            self.gas = ctypes.c_short(data[1]*256 + data[0]).value
            if self.verbose:
                print "GAS", self.gas

    def update_encoders(self, (id, data)):
        if id == 0x284:
            assert len(data) == 4, data
            arr = [data[2*i+1]*256 + data[2*i] for i in xrange(2)]
            if self.prev_enc_raw is not None:
                diffL = sint16_diff(arr[0], self.prev_enc_raw[0])
                diffR = sint16_diff(arr[1], self.prev_enc_raw[1])

                if abs(diffL) > 128:
                    print "ERR-L\t{}\t{}\t{}".format(self.dist_left_raw, self.prev_enc_raw[0], arr[0])
                else:
                    self.dist_left_raw += diffL

                if abs(diffR) > 128:
                    print "ERR-R\t{}\t{}\t{}".format(self.dist_right_raw, self.prev_enc_raw[1], arr[1])
                else:
                    self.dist_right_raw += diffR

                speed = (diffL + diffR)/2.0
                if abs(speed) < 100:
                    self.speed_raw = speed
                self.speed_arr = self.speed_arr[1:] + [self.speed_raw]
                FRAC = 0.2
                self.speed_average = FRAC*self.speed_raw + (1 - FRAC)*self.speed_average
                if self.verbose:
                    print "SPEED", self.time, self.speed_raw, self.speed_average
            self.prev_enc_raw = arr
            if self.verbose:
                print "ENC\t{}\t{}\t{}".format(self.time, self.dist_left_raw, self.dist_right_raw)
        if id == 0x184:  # change report
            assert len(data) == 1, data
#            print "CHANGE", data[0] & 0x3, (data[0] >> 2) & 0x3, self.dist_right_raw

    def update_buttons(self, (id, data)):
        if id == 0x185:
            print "DATA", data
            self.buttons_and_LEDs = data[0]
            self.can.sendData(0x205, [(0x0F & data[0])<<4])

    def update_wheel_angle_status(self, (id, data)):
        if id == 0x182:
            assert(len(data) == 2) 
            self.wheel_angle_raw = ctypes.c_short(data[1]*256 + data[0]).value
            if self.verbose:
                print "WHEEL", self.time, self.wheel_angle_raw, self.desired_wheel_angle_raw

    def update_bumpers(self, (id, data)):
        if id == 0x183:
            assert len(data) == 1, data
            assert data[0] & 0x10 == 0x10, data  # reserved unused bit
            self.bumpers = data[0]

    def update(self, packet):
        self.update_gas_status(packet)
        self.update_encoders(packet)
        self.update_wheel_angle_status(packet)
        self.update_buttons(packet)
        self.update_bumpers(packet)

    def set_time(self, time):
        if (int(self.time * SPEED_UPDATE_CONTROL_FREQ) != 
            int(time * SPEED_UPDATE_CONTROL_FREQ)):
            speed = sum(self.speed_arr)
            if self.verbose:
                print 'ref speed at', time, speed, self.last_sent_speed_cmd
            if self.desired_speed_raw is not None and self.valid_speed_ref:
                if speed + SPEED_TOLERANCE < self.desired_speed_raw:
                    self.cmd = min(self.last_sent_speed_cmd + GAS_STEP, MAX_GAS_LIMIT)
                elif speed - SPEED_TOLERANCE > self.desired_speed_raw:
                    self.cmd = max(self.last_sent_speed_cmd - GAS_STEP, MIN_GAS_LIMIT)
                if self.verbose:
                    print "-->", self.cmd
            self.valid_speed_ref = True  # ignore first, but accept following measurements

        self.time = time

    def _send_desired_gas(self, position):
        self.last_sent_speed_cmd = position
        self.can.sendData(0x201, [position & 0xFF, (position>>8)&0xFF])

    def send_speed(self):  # and turning commands
        if self.cmd == 'go':
            self._send_desired_gas(GO_LIMIT)
            self.cmd = None

        elif self.cmd == 'go_back':
            self._send_desired_gas(GO_BACK_LIMIT)
            self.cmd = None

        elif self.cmd == 'stop':
            self._send_desired_gas((CENTER_GAS_MIN + CENTER_GAS_MAX)/2)
            self.cmd = None

        elif self.cmd is not None:
            self._send_desired_gas(self.cmd)
            self.cmd = None

        if self.desired_wheel_angle_raw is not None and self.wheel_angle_raw is not None:
            if abs(self.desired_wheel_angle_raw - self.wheel_angle_raw) > TURN_TOLERANCE:
                if self.desired_wheel_angle_raw > self.wheel_angle_raw:
                    self.can.sendData(0x202, [5])  # left
                else:
                    self.can.sendData(0x202, [0x6])  # right
            else:
                self.can.sendData(0x202, [0])
                self.desired_wheel_angle_raw = None

    def send_LEDs(self):
        """Send command to set new LEDs status"""
        if self.cmd_LEDs is not None:
            self.can.sendData(0x205, [self.cmd_LEDs])

# vim: expandtab sw=4 ts=4
