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
PULSE_STEP = 500
SLOW_SPEED_MIN = 1.0
SLOW_SPEED_MAX = 2.0


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
        self.gas_count = 0  # compute average between SYNC
        self.gas_sum = 0
        self.gas_min_max = 0x10000, 0
        self.filteredGas = None
        self.desired_gas = None
        self.last_gas_dx = 0  # "derivative" for debouncing
        self.cmd = None
        self.no_change_time = None

        self.prev_enc_raw = None
        self.dist_left_raw = 0
        self.dist_right_raw = 0
        self.speed_raw = 0
        self.speed_average = 0

        self.wheel_angle_raw = None
        self.desired_wheel_angle_raw = None

    def go(self):
        self.cmd = 'go'

    def go_slowly(self):
        self.cmd = 'go_slowly'

    def stop(self):
        self.cmd = 'stop'

    def set_turn_raw(self, raw_angle):
        self.desired_wheel_angle_raw = raw_angle

    def stop_turn(self):
        "immediately stop turning = close valves"
        self.can.sendData(0x202, [0])
        self.desired_wheel_angle_raw = None

    def update_gas_status(self, (id, data)):
        # note partial duplicity with johndeere.py
        if id == 0x281:
            # obsolete, remove it
            assert( len(data)>=8 ) 
            self.gas = data[1]*256 + data[0]
            self.gas_count += 1
            self.gas_sum += self.gas
            self.gas_min_max = (min(self.gas_min_max[0], self.gas), max(self.gas_min_max[1], self.gas))

        if id == 0x181:
            assert len(data)==2, data
            self.gas = ctypes.c_short(data[1]*256 + data[0]).value
#            print "GAS", self.gas
            self.gas_count += 1
            self.gas_sum += self.gas
            self.gas_min_max = (min(self.gas_min_max[0], self.gas), max(self.gas_min_max[1], self.gas))

        elif id == 0x80:
            if self.gas_count > 0:
                self.filteredGas = self.gas_sum/float(self.gas_count)

            if self.verbose:
                if self.gas_count > 0:
                    print "SYNC", self.gas_count, self.gas_min_max, self.gas_sum/float(self.gas_count)
                else:
                    print "SYNC 0"
            self.gas_count = 0
            self.gas_sum = 0
            self.gas_min_max = 0x10000, 0

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

    def update_wheel_angle_status(self, (id, data)):
        if id == 0x182:
            assert(len(data) == 2) 
            self.wheel_angle_raw = ctypes.c_short(data[1]*256 + data[0]).value
            if self.verbose:
                print "WHEEL", self.time, self.wheel_angle_raw, self.desired_wheel_angle_raw

    def update(self, packet):
        self.update_gas_status(packet)
        self.update_encoders(packet)
        self.update_wheel_angle_status(packet)

    def set_time(self, time):
        self.time = time

    def send_desired_gas(self, position):
        self.can.sendData(0x201, [position & 0xFF, (position>>8)&0xFF])

    def send_speed(self):  # and turning commands
        if self.desired_gas is not None:
            assert 0
            # make sure the desired value is in safe=slow forward range
            assert CENTER_GAS_MIN <= self.desired_gas <= GO_LIMIT, self.desired_gas
            if self.filteredGas < self.desired_gas - PULSE_STEP and self.last_gas_dx >= 0:
                self.can.sendData(0x201, [0xC])  # pulse forward
                self.last_gas_dx = 1
                self.no_change_time = self.time + 0.5
            elif self.filteredGas > self.desired_gas + PULSE_STEP and self.last_gas_dx <= 0:
                self.can.sendData(0x201, [3])  # pulse backward
                self.last_gas_dx = -1
                self.no_change_time = self.time + 0.5
            else:
                self.can.sendData(0x201, [0])
                if self.no_change_time is None or self.no_change_time >= self.time:
                    self.last_gas_dx = 0
                    self.no_change_time = None

        # REFACTORING NEEDED coliding control methods (!!!)

        if self.cmd == 'go':
            self.send_desired_gas(GO_LIMIT)
            self.cmd = None
#            if self.filteredGas < GO_LIMIT:
#                self.can.sendData(0x201, [0xC])  # pulse forward
#            else:
#                self.can.sendData(0x201, [0])
#                self.cmd = None

        elif self.cmd == 'stop':
            print self.filteredGas
            self.send_desired_gas((CENTER_GAS_MIN + CENTER_GAS_MAX)/2)
            self.cmd = None
#            if self.filteredGas < CENTER_GAS_MIN:
#                self.can.sendData(0x201, [0xC])  # pulse forward
#            elif self.filteredGas > CENTER_GAS_MAX:
#                self.can.sendData(0x201, [3])  # pulse backward
#            else:
#                self.can.sendData(0x201, [0])
#                self.cmd = None
#                print "stop STOP"

        if self.desired_wheel_angle_raw is not None and self.wheel_angle_raw is not None:
            if abs(self.desired_wheel_angle_raw - self.wheel_angle_raw) > TURN_TOLERANCE:
                if self.desired_wheel_angle_raw > self.wheel_angle_raw:
                    self.can.sendData(0x202, [5])  # left
                else:
                    self.can.sendData(0x202, [0x6])  # right
            else:
                self.can.sendData(0x202, [0])
                self.desired_wheel_angle_raw = None



# vim: expandtab sw=4 ts=4 

