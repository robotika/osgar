#!/usr/bin/python
"""
  Wrapper for modified tractor John Deer X300R for autonomous driving.
  usage:
       ./johndeer.py <task> [<metalog> [<F>]]
"""
import sys
from can import CAN, ReplayLogInputsOnly
from time import sleep

PULSE_DURATION = 0.3  #0.5  # seconds

CENTER_GAS_MIN = 14500
CENTER_GAS_MAX = 16500

GO_LIMIT = 18000

class JohnDeer(object):
    UPDATE_TIME_FREQUENCY = 5.0  #20.0  # Hz 

    def __init__(self, can=None):
        if can is None:
            self.can = CAN()
            self.can.resetModules()
        else:
            self.can = can
        self.time = 0.0
        self.gas = None
        self.buttonGo = None
        self.desired_speed = 0.0
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

    def update_gas_status(self, (id, data)):
        if id == 0x281:
            assert( len(data)>=8 ) 
            self.gas = data[1]*256 + data[0]
            self.buttonGo = (data[-1] > 64)

    def update_encoders(self, packet):
        pass

    def update_emergency_stop(self, packet):
        pass

    def send_speed(self):
        pass

    def pulse_forward(self, duration=PULSE_DURATION):
        print "PULSE FORWARD", duration
        self.can.sendData(0x201, [0xC])
        sleep(duration)
        self.can.sendData(0x201, [0])

    def pulse_backward(self, duration=PULSE_DURATION):
        print "PULSE BACKWARD", duration
        self.can.sendData(0x201, [3])
        sleep(duration)
        self.can.sendData(0x201, [0])

    def update(self):
        while True:
            packet = self.can.readPacket()
            self.update_encoders(packet)
            self.update_gas_status(packet)
            self.update_emergency_stop(packet)
            self.check_modules(packet)
            for (name,e) in self.extensions:
                e(self, packet[0], packet[1])
            
            # make sure that all updates get also termination SYNC (0x80)
#            print packet
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

    def stop(self):
        "send stop command and make sure robot really stops"
        self.desired_speed = 0.0
        for i in xrange(10):
            self.update()
            # TODO verify encoders/motion

def center(robot):
  print "CENTER"
  for i in xrange(10):
    start_time = robot.time
    arr = []
    while robot.time - start_time < 1.0:
        robot.update()
        print robot.time, robot.gas
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
    print "!!! GO !!!"

def go(robot):
    print "GO"
    for i in xrange(10):
        start_time = robot.time
        arr = []
        while robot.time - start_time < 1.0:
            robot.update()
            print robot.time, robot.gas
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
        robot = JohnDeer()
    else:
        robot = JohnDeer(can=CAN(ReplayLogInputsOnly(replay_filename)))
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

