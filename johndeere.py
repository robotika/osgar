#!/usr/bin/python
"""
  Wrapper for modified tractor John Deer X300R for autonomous driving.
  usage:
       ./johndeere.py <task> [<metalog> [<F>]]
"""
import sys
import math
from can import CAN, ReplayLogInputsOnly, ReplayLog
from canproxy import CANProxy
from sdoplg import ReadSDO, WriteSDO 
from apyros.metalog import MetaLog, disableAsserts


# meters per single encoder tick
ENC_SCALE = 2*3.39/float(252 + 257)

TURN_ANGLE_OFFSET = math.radians(5.5)
TURN_SCALE = 0.0041


# TODO move inside or remove when CAN module is upgraded
def setup_faster_update(can):
    reader = ReadSDO( 1, 0x1801, 5 )
    for packet in reader.generator():
        if packet != None:
            can.sendData( *packet )
        reader.update( can.readPacket() )
    print "RESULT DATA (before):", reader.result 

    writer = WriteSDO( 1, 0x1801, 5, [5, 0] )
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
#    writer = WriteSDO( 0x7F, 0x2100, 1, [3] )
#    for cmd in writer.generator():
#        if cmd:
#            can.sendData( *cmd )
#        writer.update( can.readPacket() ) 




class JohnDeere(object):
    UPDATE_TIME_FREQUENCY = 5.0  #20.0  # Hz 

    def __init__(self, can=None):
        if can is None:
            self.can = CAN()
            self.can.resetModules()
        else:
            self.can = can
        self.canproxy = CANProxy(self.can)
        self.time = 0.0
        self.buttonGo = None  # TODO currently not available (!)
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

    def update(self):
        while True:
            packet = self.can.readPacket()
            self.canproxy.update(packet)
#            self.update_emergency_stop(packet)
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
        self.canproxy.set_time(self.time)
        self.canproxy.send_speed()
        self.send_ball_dispenser()

    def set_desired_speed(self, speed):
        """set desired speed in meters per second.
        speed = None ... disable speed control
        ... can be called in every cycle without side-effects
        """
        self.canproxy.set_desired_speed_raw(int(speed/ENC_SCALE))

    def set_desired_steering(self, angle):
        """set desired steering angle of left wheel"""
        # angle = sensors['steering'] * TURN_SCALE + TURN_ANGLE_OFFSET  # radians
        raw = (angle - TURN_ANGLE_OFFSET)/TURN_SCALE
        self.canproxy.set_turn_raw(int(raw))

    def stop(self):
        "send stop command and make sure robot really stops"
        self.canproxy.stop()
        for i in xrange(10):
            self.update()
            # TODO verify encoders/motion


def wait_for_start(robot):
    print "WAIT FOR START"
    while not robot.buttonGo:
        robot.update()
    print "STARTED ..."


def self_test(metalog):
    assert metalog is not None
    can_log_name = metalog.getLog('can')
    if metalog.replay:
        if metalog.areAssertsEnabled():
            can = CAN(ReplayLog(can_log_name), skipInit=True)
        else:
            can = CAN(ReplayLogInputsOnly(can_log_name), skipInit=True)
    else:
        can = CAN()
        can.relog(can_log_name)
    can.resetModules(configFn=setup_faster_update)
    robot = JohnDeere(can=can)
    robot.UPDATE_TIME_FREQUENCY = 20.0  # TODO change internal and integrate setup

    robot.canproxy.stop()
#    wait_for_start(robot)
    robot.set_desired_speed(0.5)
    start_time = robot.time
    robot.canproxy.set_turn_raw(0)
    robot.canproxy.go()
    start_dist = robot.canproxy.dist_left_raw + robot.canproxy.dist_right_raw
    arr = []
    while robot.time - start_time < 10.0:
        robot.update()
        arr.append(robot.canproxy.gas)
        dist = ENC_SCALE*(robot.canproxy.dist_left_raw + robot.canproxy.dist_right_raw 
                          - start_dist)/2.0
        if dist > 1.0:
            print "Dist OK at {}s".format(robot.time - start_time), sorted(arr)[len(arr)/2]
            break
    print dist
    robot.stop()
    robot.wait(3.0)
    dist = ENC_SCALE*(robot.canproxy.dist_left_raw + robot.canproxy.dist_right_raw 
                      - start_dist)/2.0
    print dist
    print

    robot.canproxy.go_back()
    start_time = robot.time
    start_dist = robot.canproxy.dist_left_raw + robot.canproxy.dist_right_raw
    arr = []
    while robot.time - start_time < 10.0:
        robot.update()
        arr.append(robot.canproxy.gas)
        dist = ENC_SCALE*(robot.canproxy.dist_left_raw + robot.canproxy.dist_right_raw 
                          - start_dist)/2.0
        if dist < -1.0:
            print "Dist back OK at {}s".format(robot.time - start_time), sorted(arr)[len(arr)/2]
            break
    print dist


#        print robot.time, robot.canproxy.gas
#        if not robot.buttonGo:
#            print "STOP!"
#            break
    robot.canproxy.stop_turn()
    robot.stop()
    robot.wait(3.0)
    dist = ENC_SCALE*(robot.canproxy.dist_left_raw + robot.canproxy.dist_right_raw 
                      - start_dist)/2.0
    print dist

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print __doc__
        sys.exit(2)
    metalog=None
    if 'meta_' in sys.argv[1]:
        metalog = MetaLog(filename=sys.argv[1])
    elif len(sys.argv) > 2:
        metalog = MetaLog(filename=sys.argv[2])
    if len(sys.argv) > 2 and sys.argv[-1] == 'F':
        disableAsserts()    
    if metalog is None:
        metalog = MetaLog()
    self_test(metalog)

# vim: expandtab sw=4 ts=4 

