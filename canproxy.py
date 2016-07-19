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

    def go(self):
        self.cmd = 'go'

    def stop(self):
        self.cmd = 'stop'

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



    def pulse_forward(self, duration=PULSE_DURATION):
        print "PULSE FORWARD", duration
        self.can.sendData(0x201, [0xC])
        self.wait(duration)
        self.can.sendData(0x201, [0])

    def pulse_backward(self, duration=PULSE_DURATION):
        print "PULSE BACKWARD", duration
        self.can.sendData(0x201, [3])
        self.wait(duration)
        self.can.sendData(0x201, [0])

    def pulse_left(self, duration):
        print "PULSE LEFT", duration
        self.can.sendData(0x20C, [9])
        self.wait(duration)
        self.can.sendData(0x20C, [0])

    def pulse_right(self, duration):
        print "PULSE RIGHT", duration
        self.can.sendData(0x20C, [0xA])
        self.wait(duration)
        self.can.sendData(0x20C, [0])


def center(robot):
  print "CENTER"
  if robot.filteredGas > GO_LIMIT:
    robot.pulse_backward(0.1)
  for i in xrange(20):
    start_time = robot.time
    arr = []
    while robot.time - start_time < 0.2:
        robot.update()
        arr.append(robot.gas)
    assert len(arr) > 0
    avr = sum(arr)/float(len(arr))
    print avr
    if avr < CENTER_GAS_MIN:
        robot.pulse_forward(0.05)
    elif avr > CENTER_GAS_MAX:
        robot.pulse_backward(0.05)
    else:
        break
  print "CENTER DONE"


def go(robot):
    print "GO"
    for i in xrange(10):
        start_time = robot.time
        arr = []
        while robot.time - start_time < 1.0:
            robot.update()
            arr.append(robot.gas)
        assert len(arr) > 0
        avr = sum(arr)/float(len(arr))
        print avr
        if avr < GO_LIMIT:
            robot.pulse_forward(0.1)
        else:
            break
    print "RUNNING!"

# vim: expandtab sw=4 ts=4 

