#!/usr/bin/python
"""
  Remote control of John Deere via WiFi
  usage:
       ./driver.py <notes> | [<metalog> [<F>]]
"""

import sys
import math
from socket import timeout
from threading import Thread,Event,Lock 

from apyros.metalog import MetaLog, disableAsserts
from apyros.sourcelogger import SourceLogger
from gps import DummyGPS as DummySensor  # TODO move to apyros, as mock


from can import CAN, DummyMemoryLog, ReplayLogInputsOnly, ReplayLog
from johndeere import JohnDeere, setup_faster_update, ENC_SCALE

HOST = "192.168.1.201"
PORT = 5555

SOCKET_TIMEOUT = 0.2


class RemoteThread(Thread):

    def __init__(self, soc): 
        Thread.__init__(self)
        self.setDaemon(True)
        self.lock = Lock()
        self.shouldIRun = Event()
        self.shouldIRun.set()
        self.soc = soc
        self.data = None
        self.started = False

    def run(self):
        while self.shouldIRun.isSet():
            try:
                self.data = self.soc.recv(2000)
                self.started = True
            except timeout:
                if self.started:
                    self.data = 'STOP\n'
                else:
                    self.data = None

    def requestStop(self):
        self.shouldIRun.clear() 

    def get_data(self):
        ret = self.data
        self.data = None
        return ret


def remote_data_extension(robot, id, data):
    if id=='remote':
        robot.remote_data = data


def drive_remotely(metalog):
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

    robot.localization = None  # TODO

    soc = metalog.createLoggedSocket('remote', headerFormat=None)  # TODO fix headerFormat
    soc.bind( ('',PORT) )
    if not metalog.replay:
        soc.soc.setblocking(0)
        soc.soc.settimeout( SOCKET_TIMEOUT )
    
    remote_cmd_log_name = metalog.getLog('remote_cmd')
    if metalog.replay:
        robot.remote = DummySensor()
        function = SourceLogger(None, remote_cmd_log_name).get
    else:
        robot.remote = RemoteThread(soc)
        function = SourceLogger(robot.remote.get_data, remote_cmd_log_name).get

    robot.remote_data = None
    robot.register_data_source('remote', function, remote_data_extension) 
    robot.remote.start()

    robot.canproxy.stop()
    robot.canproxy.set_turn_raw(0)

    print "Waiting for remote commands ..."
    while robot.remote_data is None:
        robot.update()
    print "received", robot.remote_data.strip()

    moving = False
    turning = False
    while robot.remote_data != 'END\n':
        robot.update()
        if robot.remote_data == 'STOP\n' and (moving or turning):
            if moving:
                robot.canproxy.stop()
                print "STOP"
                moving = False
            if turning:
                robot.canproxy.stop_turn()
                print "STOP turn"
                turning = False
        elif robot.remote_data == 'UP\n' and not moving:
            robot.canproxy.go()
            print "GO"
            moving = True
        elif robot.remote_data == 'DOWN\n' and not moving:
            robot.canproxy.go_back()
            print "GO back"
            moving = True
        elif robot.remote_data == 'LEFT\n':
            robot.canproxy.set_turn_raw(200)
            print "Left"
            turning = True
        elif robot.remote_data == 'RIGHT\n':
            robot.canproxy.set_turn_raw(-200)
            print "Right"
            turning = True

    print "received", robot.remote_data.strip()

    robot.canproxy.stop_turn()
    robot.remote.requestStop()
    robot.wait(3.0)


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
    drive_remotely(metalog)

# vim: expandtab sw=4 ts=4 

