#!/usr/bin/python
"""
  Wrapper for modified Spider 3Rider for autonomous driving.
  usage:
       ./spider3.py <task> [<metalog> [<F>]]
"""
import sys
import struct

from can import CAN, ReplayLogInputsOnly, ReplayLog
from apyros.metalog import MetaLog, disableAsserts

from johndeere import EmergencyStopException  # move this to abstract robot


def emergency_stop_extension(robot, id, data):
    if (robot.status_word is not None and 
        robot.status_word & robot.EMERGENCY_STOP == 0):
        print("raising EMERGENCY EXCEPTION!")
        raise EmergencyStopException()


def fix_range(value):
    "into <-256, +256) interval"
    if value < -256:
        value += 512
    elif value >= 256:
        value -= 512
    return value


class Spider3(object):
    UPDATE_TIME_FREQUENCY = 20.0  # Hz 

    # status word
    EMERGENCY_STOP = 0x0001
    DEVICE_READY = 0x0002  # also releaseState
    MASK_HEARTBEAT = 0x8000

    def __init__(self, can=None):
        if can is None:
            self.can = CAN()
#            self.can.resetModules()
        else:
            self.can = can
        self.time = 0.0
        self.status_word = None  # not defined yet
        self.wheel_angles = [None]*4  # original CAN data
        self.drive_status = [None]*4
        self.zero_steering = None  # zero position of all 4 wheels
        self.alive = 0  # togle with 128
        self.speed_cmd = [0, 0]
        self.status_cmd = 3
        self.extensions = []
        self.data_sources = []
        self.threads = []
        self.modulesForRestart = []
        self.led = 0
        self.led_time = 0.0
#        self.can.sendOperationMode()
  
    def __del__(self):
        pass
#        self.can.sendPreoperationMode() 

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
                    print("RESET", moduleId)
                    self.can.sendData( 0, [129,moduleId] )
#                if moduleId in [0x01, 0x02]:
#                    if (0x180 | moduleId) in self.encData:
#                      # The encoder information is invalid during a reset
#                      del self.encData[0x180 | moduleId]
            elif data[0] == 127: # restarted and in preoperation
                print("SWITCH TO OPERATION", moduleId)
                self.can.sendData( 0, [1,moduleId] ) 
            elif moduleId in self.modulesForRestart:
                print("RUNNING", moduleId)
                self.modulesForRestart.remove(moduleId)

    def register_data_source(self, name, function, extension=None):
        self.data_sources.append((name, function))
        if extension is not None:
            self.extensions.append((name, extension))

    def wait(self, duration):
        start_time = self.time
        while self.time - start_time < duration:
            self.update()

    def update_status_word(self, packet):
        msg_id, data = packet
        if msg_id == 0x200:
            assert len(data) == 2, packet
            prev = self.status_word
            self.status_word = struct.unpack('H', bytearray(data))[0]
            if prev is not None:
                diff = prev ^ self.status_word
                is_alive = diff & self.MASK_HEARTBEAT
                if is_alive == 0:
                    # make sure nothing is missing (for now)
                    print("\n!!!Spider is DEAD!!! (or data are missing)\n")
                if (diff & 0x7FFF) != 0:
                    print("Status %d -> %d" % 
                            (prev & 0x7FFF, self.status_word & 0x7FFF))

        elif msg_id == 0x201:
            assert len(data) == 8, packet
            prev = self.wheel_angles
            self.wheel_angles = struct.unpack('HHHH', bytearray(data))
            if prev != self.wheel_angles and self.zero_steering is not None:
                # note, that self.zero_steering has 200ms update rate,
                # i.e. it does not have to be immediately defined
                print('Wheels: %.2f' % self.time, [fix_range(a - b) for a, b in zip(self.wheel_angles, self.zero_steering)])

        elif msg_id == 0x202:
            assert len(data) == 8, packet  # drive status + zero positions
            self.drive_status = struct.unpack('HHHH', bytearray(data))
            drive = self.drive_status[0] - self.drive_status[2], self.drive_status[3] - self.drive_status[1]
            if abs(drive[0]) + abs(drive[1]) > 10:
                print('Drive:', drive, abs(drive[0]) + abs(drive[1]))

        elif msg_id == 0x203:
            assert len(data) == 8, packet
            prev = self.zero_steering
            self.zero_steering = struct.unpack('HHHH', bytearray(data))
            # make sure that calibration did not change during program run
            assert prev is None or prev == self.zero_steering, (prev, self.zero_steering)

    def send_speed(self):
        self.can.sendData(0x400, [self.status_cmd, self.alive])
        self.can.sendData(0x401, self.speed_cmd)  # drive control data drive, steering
        self.alive = 128 - self.alive

    def send_LED(self):
        self.can.sendData(0x410, [1, 2, 3, 4, 5, 6, 7, self.led])
        if self.time - self.led_time > 0.5:
            self.led = 1 - self.led
            self.led_time = self.time

    def set_raw_speed(self, fwd_rev_drive, left_right):
        print('set_raw_speed', fwd_rev_drive, left_right)
        self.speed_cmd = [fwd_rev_drive, left_right]

    def update(self):
        while True:
            packet = self.can.readPacket()
            self.check_modules(packet)
            self.update_status_word(packet)
            for (name,e) in self.extensions:
                e(self, packet[0], packet[1])
            
            # make sure that all updates get also termination SYNC (0x80)
            # there is no SYNC on Spider3 yet, use 0x200 for now
            if packet[0] == 0x200:
                break
        
        # send data related to other sources
        for (id,fce) in self.data_sources:
            data = fce()
            if data != None:
                for (name,e) in self.extensions:
                    e(self, id, data)

        self.time += 1.0/self.UPDATE_TIME_FREQUENCY  
        self.send_speed()
        self.send_LED()

    def stop(self):
        "send stop command and make sure robot really stops"
        self.set_raw_speed(0, 0)
        for i in range(10):
            self.update()
            # TODO verify encoders/motion


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
    #can.resetModules()
    robot = Spider3(can=can)
    robot.localization = None

    try:
        robot.extensions.append(('emergency_stop', emergency_stop_extension))
        run_it(robot)
    except EmergencyStopException:
        print("Emergency STOP Exception!")

    robot.extensions = []
    robot.stop()
    robot.wait(3.0)

    
def run_it(robot):

    # wait for unblocked remote STOP
    while robot.status_word is None or robot.status_word & robot.EMERGENCY_STOP == 0:
        robot.update()

    if robot.status_word is None or robot.status_word & robot.DEVICE_READY == 0:
        robot.update()
        while robot.status_word is None or robot.status_word & robot.DEVICE_READY == 0:
           robot.update()
        # add extra time for start-up machinery?
        print("Waiting 5s to start engine!")
        robot.wait(5.0)

    robot.set_raw_speed(0x80 + 127, 0x80 + 50)
    robot.wait(10.0)

    robot.set_raw_speed(0, 0)
    robot.wait(2.0)

    robot.set_raw_speed(0x80 + 127, 50)
    robot.wait(5.0)

    robot.set_raw_speed(0, 0)
    robot.wait(2.0)



if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
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

