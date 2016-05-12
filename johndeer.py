#!/usr/bin/python
"""
  Wrapper for modified tractor John Deer X300R for autonomous driving.
  usage:
       ./johndeer.py <task> [<metalog> [<F>]]
"""
import sys
from can import CAN, DummyMemoryLog

class JohnDeer(object):
    UPDATE_TIME_FREQUENCY = 20.0  # Hz 

    def __init__(self, can=None):
        if can is None:
            self.can = CAN()
            self.can.resetModules()
        else:
            self.can = can
        self.time = 0.0
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

    def update_encoders(self, packet):
        pass

    def update_emergency_stop(self, packet):
        pass

    def send_speed(self):
        pass

    def update(self):
        while True:
            packet = self.can.readPacket()
            self.update_encoders(packet)
            self.update_emergency_stop(packet)
            self.check_modules(packet)
            for (name,e) in self.extensions:
                e(self, packet[0], packet[1])
            
            # make sure that all updates get also termination SYNC (0x80)
            if packet[0] == 0x80:  
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


def self_test():
    com = DummyMemoryLog()
    com.data += [0x80>>3, 0]*10000  # just test/hack without CAN module
    robot = JohnDeer(can=CAN(com=com))
    robot.desired_speed = 0.5
    start_time = robot.time
    while robot.time - start_time < 10.0:
        robot.update()
    robot.stop()

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print __doc__
        sys.exit(2)
    self_test()

# vim: expandtab sw=4 ts=4 

