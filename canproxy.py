"""
  CAN bus proxy, i.e wrapper for some task which will be handled by CAN later.
  In particular:
      - state machine for gas pedal control
      - timing for turn (without feedback)
"""

PULSE_DURATION = 0.3  #0.5  # seconds

CENTER_GAS_MIN = 14500
CENTER_GAS_MAX = 16500

GO_LIMIT = 19000

SCALE_NEW = 0.5  # 0.0 < x < 1.0


class CANProxy:

    def __init__(self, can):
        self.can = can
        self.time = 0.0
        self.gas = None
        self.filteredGas = None
        self.cmd = None
        self.turn_cmd = None
        self.turn_stop_time = 0.0


    def go(self):
        self.cmd = 'go'

    def stop(self):
        self.cmd = 'stop'

    def pulse_left(self, duration):
        self.turn_cmd = 'left'
        self.turn_stop_time = self.time + duration

    def pulse_right(self, duration):
        self.turn_cmd = 'right'
        self.turn_stop_time = self.time + duration


    def update_gas_status(self, (id, data)):
        # note partial duplicity with johndeere.py
        if id == 0x281:
            assert( len(data)>=8 ) 
            self.gas = data[1]*256 + data[0]
            if self.filteredGas is None:
                self.filteredGas = self.gas
            else:
                self.filteredGas = SCALE_NEW*self.gas + (1.0 - SCALE_NEW)*self.filteredGas

    def update(self, packet):
        self.update_gas_status(packet)

    def set_time(self, time):
        self.time = time

    def send_speed(self):
        if self.cmd == 'go':
            if self.filteredGas < GO_LIMIT:
                self.can.sendData(0x201, [0xC])  # pulse forward
            else:
                self.can.sendData(0x201, [0])
                self.cmd = None

        elif self.cmd == 'stop':
            if self.filteredGas < CENTER_GAS_MIN:
                self.can.sendData(0x201, [0xC])  # pulse forward
            elif self.filteredGas > CENTER_GAS_MAX:
                self.can.sendData(0x201, [3])  # pulse backward
            else:
                self.can.sendData(0x201, [0])
                self.cmd = None

        if self.turn_cmd is not None:
            if self.turn_stop_time <= self.time:
                self.can.sendData(0x20C, [0])
                self.turn_cmd = None
            else:
                if self.turn_cmd == 'left':
                    self.can.sendData(0x20C, [9])
                elif self.turn_cmd == 'right':
                    self.can.sendData(0x20C, [0xA])
                else:
                    assert 0, self.turn_cmd  # unknown turn_cmd

# vim: expandtab sw=4 ts=4 

