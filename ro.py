#!/usr/bin/python
"""
  RoboOrienteering contest with John Deere
  usage:
       ./ro.py <task> [<metalog> [<F>]]
"""
import sys
from can import CAN, DummyMemoryLog, ReplayLogInputsOnly, ReplayLog
from gps import GPS
from gps import DummyGPS as DummySensor  # TODO move to apyros, as mock
from velodyne import VelodyneThread
from johndeere import (JohnDeere, center, go, wait_for_start, 
                       setup_faster_update)
from apyros.metalog import MetaLog, disableAsserts
from apyros.sourcelogger import SourceLogger


class RoboOrienteering:
    def __init__(self, robot, configFilename, verbose=False):
        self.robot = robot
        self.mount_sensor(GPS)

    def mount_sensor(self, sensor_factory):
        """Register sensor as robot extension"""
        # sensor specific start-up params
        # global start, global terminate
        pass
"""
EduroMaxi code:
  def attachGPS(self):
    if self.replyLog is None:
      self.gps = GPS( verbose=0 )
      name = timeName( "logs/src_gps_", "log" )
      if self.metaLog:
        self.metaLog.write("GPSLOG:\t" + name + "\n")
        self.metaLog.flush()
      self.registerDataSource( 'gps', SourceLogger( self.gps.coord, name ).get )
    else:
      self.gps = DummyGPS()
      if self.metaLog:
        gpsSrcLog = self.metaLogName( "GPSLOG:" )
      else:
        gpsSrcLog = self.replyLog[:-18]+"src_gps_"+self.replyLog[-17:]
      print "GPSLOG:", gpsSrcLog
      self.registerDataSource( 'gps', SourceLogger( None, gpsSrcLog ).get )
    self.gpsData = None
    self.addExtension( gpsDataExtension ) 
"""

def gps_data_extension(robot, id, data):
    if id=='gps':
        robot.gps_data = data
        if robot.localization is not None:
            robot.localization.updateGPS(data) 

def ver0(metalog):
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

    # mount_sensor(GPS, robot, metalog)
    gps_log_name = metalog.getLog('gps')
    print gps_log_name
    if metalog.replay:
        robot.gps = DummySensor()
        function = SourceLogger(None, gps_log_name).get
    else:
        robot.gps = GPS(verbose=0)
        function = SourceLogger(robot.gps.coord, gps_log_name).get
    robot.gps_data = None
    robot.register_data_source('gps', function, gps_data_extension) 


    robot.gps.start()

    center(robot)
    wait_for_start(robot)
    robot.desired_speed = 0.5
    start_time = robot.time
    go(robot)
    while robot.time - start_time < 30*60:  # RO timelimit 30 minutes
        robot.update()
        print robot.time, robot.gas, robot.gps_data
        if not robot.buttonGo:
            print "STOP!"
            break
    center(robot)
    robot.gps.requestStop()


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
    ver0(metalog)


# There are too many TODOs at the moment, including basic CAN modules integration (EmergencySTOP etc)
# Common launcher will be postponed.
#if __name__ == "__main__":
#    from johndeere import JohnDeere
#    import launcher
#    launcher.launch(sys.argv, JohnDeere, RoboOrienteering, configFn=setup_faster_update) 

# vim: expandtab sw=4 ts=4 

