#!/usr/bin/python
"""
  Wrapper for modified tractor John Deer X300R for autonomous driving.
  usage:
       ./johndeere.py <task> [<metalog> [<F>]]
"""
import sys
import math
from can import CAN, ReplayLogInputsOnly
from sdoplg import ReadSDO, WriteSDO 

PULSE_DURATION = 0.3  #0.5  # seconds

CENTER_GAS_MIN = 14500
CENTER_GAS_MAX = 16500

GO_LIMIT = 19000

SCALE_NEW = 0.5  # 0.0 < x < 1.0
ACC_SMOOTHING = 0.1 # scale factor (0..1) for old measurement, 0.0 = only new measurements will be used 


# TODO move inside or remove when CAN module is upgraded
def setup_faster_update(can):
    reader = ReadSDO( 1, 0x1801, 5 )
    for packet in reader.generator():
        if packet != None:
            can.sendData( *packet )
        reader.update( can.readPacket() )
    print "RESULT DATA (before):", reader.result 

    writer = WriteSDO( 1, 0x1801, 5, [50, 0] )
    for cmd in writer.generator():
        if cmd:
            can.sendData( *cmd )
        writer.update( can.readPacket() ) 

    for packet in reader.generator():
        if packet != None:
            can.sendData( *packet )
        reader.update( can.readPacket() )
    print "RESULT DATA (after):", reader.result 

    # ball dispenser
    writer = WriteSDO( 0x7F, 0x2100, 1, [3] )
    for cmd in writer.generator():
        if cmd:
            can.sendData( *cmd )
        writer.update( can.readPacket() ) 



# TODO revise with Python struct
def sint16( data ):
    ret = data[1]*256+data[0]
    if ret > 0x8000:
        ret = ret-0x10000
    return ret 

def analyseCompass(compassRaw, compassAcc):
    # copy & paste from eduro/compass.py
    # horizontal plane is given by ax+by+cz+d=0, where (a,b,c)=compassAcc and d=0  
    # incination in Prague is 66deg, declination is 11deg
    inc = math.radians(66.0)
    vecB = (math.cos(inc), 0.0, math.sin(inc)) # handle declination afterwards

    compassOffset = (-150, 550, 2250)

    # vector multiplication
    # C = A x B = (AyBz - ByAz, AzBx-BzAx, AxBy-BxAy)
    A = compassAcc # in reality z-coordinate
    B = (compassRaw[0]-compassOffset[0], compassRaw[1]-compassOffset[1], compassRaw[2]-compassOffset[2])
    C = (A[1]*B[2]-B[1]*A[2], A[2]*B[0]-B[2]*A[0], A[0]*B[1]-B[0]*A[1])
    return int(10*math.degrees(math.atan2(C[0],C[1])))



class JohnDeere(object):
    UPDATE_TIME_FREQUENCY = 5.0  #20.0  # Hz 

    def __init__(self, can=None):
        if can is None:
            self.can = CAN()
            self.can.resetModules()
        else:
            self.can = can
        self.time = 0.0
        self.gas = None
        self.steering_angle = 0.0  # in the future None and auto-detect
        self.buttonGo = None
        self.desired_speed = 0.0
        self.filteredGas = None
        self.compass = None
        self.compassRaw = None
        self.compassAcc = None
        self.compassAccRaw = None
        self.drop_ball = False  # TODO move to ro.py only
        self.extensions = []
        self.data_sources = []
        self.modulesForRestart = []
        self.can.sendOperationMode()
  
    def __del__(self):
        self.can.sendPreoperationMode() 

    # TODO to move out, can.py??
    def check_modules(self, packet):
        # test if all modules are in Operation mode, if not restart them
        id, data = packet
        if id & 0xFF0 == 0x700: # heart beat message
            moduleId = id & 0xF
            assert( len(data) == 1 )
            if data[0] != 5:
                self.can.printPacket( id, data )
                if not moduleId in self.modulesForRestart:
                    self.modulesForRestart.append( moduleId )
                    print "RESET", moduleId
                    self.can.sendData( 0, [129,moduleId] )
#                if moduleId in [0x01, 0x02]:
#                    if (0x180 | moduleId) in self.encData:
#                      # The encoder information is invalid during a reset
#                      del self.encData[0x180 | moduleId]
            elif data[0] == 127: # restarted and in preoperation
                print "SWITCH TO OPERATION", moduleId
                self.can.sendData( 0, [1,moduleId] ) 
            elif moduleId in self.modulesForRestart:
                print "RUNNING", moduleId
                self.modulesForRestart.remove(moduleId)

    def register_data_source(self, name, function, extension=None):
        self.data_sources.append((name, function))
        if extension is not None:
            self.extensions.append((name, extension))

    def update_gas_status(self, (id, data)):
        if id == 0x281:
            assert( len(data)>=8 ) 
            self.gas = data[1]*256 + data[0]
            self.buttonGo = (data[-1] > 64)
            if self.filteredGas is None:
                self.filteredGas = self.gas
            else:
                self.filteredGas = SCALE_NEW*self.gas + (1.0 - SCALE_NEW)*self.filteredGas

    def update_encoders(self, packet):
        pass

    def update_emergency_stop(self, packet):
        pass

    def update_compass(self, (id, data)):
        if id == 0x187:
            pass
        if id == 0x387: # 3D accelerometer
            self.compassAccRaw = (sint16(data[0:2]), sint16(data[2:4]), sint16(data[4:6]) )
            if self.compassAcc is not None:
                self.compassAcc = (
                    (1.0-ACC_SMOOTHING)*self.compassAcc[0] + ACC_SMOOTHING*self.compassAccRaw[0], 
                    (1.0-ACC_SMOOTHING)*self.compassAcc[1] + ACC_SMOOTHING*self.compassAccRaw[1], 
                    (1.0-ACC_SMOOTHING)*self.compassAcc[2] + ACC_SMOOTHING*self.compassAccRaw[2])
            else:
                self.compassAcc = self.compassAccRaw  # init value for averaging 
        if id == 0x487: # 3D raw compass data
            self.compassRaw = (sint16(data[0:2]), sint16(data[2:4]), sint16(data[4:6]) )
            self.compass = analyseCompass(self.compassRaw, self.compassAcc)

    def send_speed(self):
        pass

    def send_ball_dispenser(self):
        if self.drop_ball:
            cmd = 127
        else:
            cmd = 128
        self.can.sendData(0x37F, [0, cmd, 0, 0, 0, 0, 0, 0])

    def wait(self, duration):
        start_time = self.time
        while self.time - start_time < duration:
            self.update()

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

    def update(self):
        while True:
            packet = self.can.readPacket()
            self.update_encoders(packet)
            self.update_gas_status(packet)
            self.update_compass(packet)
            self.update_emergency_stop(packet)
            self.check_modules(packet)
            for (name,e) in self.extensions:
                e(self, packet[0], packet[1])
            
            # make sure that all updates get also termination SYNC (0x80)
            if packet[0] == 0x281:  # 0x80:  
                break

        # send data related to other sources
        for (id,fce) in self.data_sources:
            data = fce()
            if data != None:
                for (name,e) in self.extensions:
                    e(self, id, data)

        self.time += 1.0/self.UPDATE_TIME_FREQUENCY  
        self.send_speed()
        self.send_ball_dispenser()

    def stop(self):
        "send stop command and make sure robot really stops"
        self.desired_speed = 0.0
        for i in xrange(10):
            self.update()
            # TODO verify encoders/motion

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


def wait_for_start(robot):
    print "WAIT FOR START"
    while not robot.buttonGo:
        robot.update()
    print "STARTED ..."

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

def self_test(replay_filename=None):
    if replay_filename is None:
        robot = JohnDeere()
    else:
        robot = JohnDeere(can=CAN(ReplayLogInputsOnly(replay_filename)))
    center(robot)
    wait_for_start(robot)
    robot.desired_speed = 0.5
    start_time = robot.time
#    robot.can.sendData(0x201, [0xC])
#    robot.pulse_forward()
    go(robot)
#    robot.pulse_backward()
    while robot.time - start_time < 333.0:
        robot.update()
        print robot.time, robot.gas
        if not robot.buttonGo:
            print "STOP!"
            break
#    robot.pulse_backward()
#    start_time = robot.time
#    while robot.time - start_time < 2.0:
#        robot.update()
#        print robot.time, robot.gas
#    robot.stop()
    center(robot)

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print __doc__
        sys.exit(2)
    if sys.argv[1].endswith('.log'):
        self_test(replay_filename=sys.argv[1])
    else:
        self_test()

# vim: expandtab sw=4 ts=4 

