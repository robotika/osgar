#!/usr/bin/python
"""
  Laser SICK LMS100-10000/TIM310
    autodetection: 192.168.1.1, ports 2111 (aux) and 2112 (host)
  usage:
      laser.py <IP|USB> [--config|<number of scans>]
"""

import sys
import socket
import time
from threading import Thread,Event,Lock

from camera import timeName  # hack
import struct


# TIM310
#import usb.core
#import usb.util

HOST = '192.168.2.23'    # The remote host
PORT = 2111             # The same port as used by the server

STX = chr(2)
ETX = chr(3)

class Laser( Thread ):
  def __init__( self, remission=False ):
    Thread.__init__(self) 
    self.setDaemon(True)
    self.lock = Lock()
    self.shouldIRun = Event()
    self.shouldIRun.set()
    self._timestamp = None  # timestamp of the last scane
    self._scanData = None 
    self._remissionData = None
    self.stopOnExit = True
    self.useRemission = remission

  def queryStatus( self ):
    return self.sendCmd( 'sRN STlms' )

  def readContaminationLevel( self ):
    return self.sendCmd( 'sRN LCMstate' )

  def queryScanConfig( self ):
    # default answer: "sRA LMPscancfg 1388 1 1388 FFF92230 225510", i.e. 
    return self.sendCmd( 'sRN LMPscancfg' )

  def setScanConfig( self ):
    assert( False ) # not implemented yet
    data = "" # TODO
    return self.sendCmd( 'sMN mLMPsetscancfg' + data )

  # THREAD code
  def run(self):
    if self.useRemission:
      print self.configureScanDataOutput()
    self.startLaser()
    while self.shouldIRun.isSet():
      self._timestamp, self._scanData, self._remissionData = self.internalScan()
    if self.stopOnExit:
      self.stopLaser()

  def scan(self):
    self.lock.acquire()
    xy = self._scanData
    self.lock.release()
    return xy

  def remission(self):
    self.lock.acquire()
    xy = self._remissionData
    self.lock.release()
    return xy

  def requestStop(self):
    self.shouldIRun.clear()
 

class LaserUSB( Laser ):
  def __init__( self, remission=False, errLog = None ):
    self.errLog = errLog
    self.dev = usb.core.find(idVendor=0x19A2, idProduct=0x5001)
    for i in xrange(10):
      try:
        self.dev.set_configuration()
        break;
      except:
        print "LaserUSB - init ERROR", i
        if self.errLog:
          self.errLog.write( "LaserUSB - init ERROR %d\n" % i )
          self.errLog.flush()
        time.sleep(0.1)
    Laser.__init__( self, remission=remission )

  def __del__( self ):
    del self.dev

  def sendCmd( self, cmd ):
    for i in xrange(10):
      try:
        self.dev.write(2|usb.ENDPOINT_OUT, "\x02"+cmd+"\x03\0", 0)
        arr = self.dev.read(1|usb.ENDPOINT_IN, 65535, timeout = 100)
        return "".join([chr(x) for x in arr[1:-1]])
      except:
        print "Laser", i
        if self.errLog:
          self.errLog.write( "LaserUSB - sndCmd ERROR %d\n" % i )
          self.errLog.flush()        
    
  def internalScan( self ):
    time.sleep(0.1)
    data = self.sendCmd( 'sRN LMDscandata' ).split()
    timestamp, dist, remission = time.time(), None, None
    if len(data) == 580:
      # TIM, hacked, probably DIST1, RSSI1
      dist = [int(x,16) for x in data[26:271+26]]
      remission = [int(x,16) for x in data[304:-5]]
      # hack, Time since the TiM3xx was switched on and the end point of the scan in micro seconds
      dist[0] = int(data[9],16) 
    else:
      pass #print "ERROR", data
    return timestamp, dist, remission

  def startLaser( self ):
    pass

  def stopLaser( self ):
    pass

  def configureScanDataOutput( self ):
    pass


def write_timestamp(f):
  """write time.time() in sec/frac into a binary file"""
  t = time.time()
  sec = int(t)
  frac = int(0x10000*(t-sec))
  f.write(struct.pack('IH', sec, frac))


class LaserIP( Laser ):
  def __init__( self, **kw ):
    self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    self.socket.connect((HOST, PORT))
    self.raw_log = open(timeName('logs/laserraw_', 'bin'), 'wb')
    self.raw_log.write(struct.pack('HH', 0xF472, 2))  # magic number + version 2 with timestamps
    write_timestamp(self.raw_log)
    self.raw_log.flush()
    self._buffer = ""
    Laser.__init__( self, **kw )
    self.sendCmd( 'sMN SetAccessMode 03 F4724744' )

  def __del__( self ):
    self.socket.close()
    self.raw_log.close()

  def receive( self ):
    data = self._buffer
    while True:
      pos = data.find(ETX)
      if pos >= 0:
        break
      data = self.socket.recv(1024)
      if len(data) > 0:
        write_timestamp(self.raw_log)
        self.raw_log.write(struct.pack('HH', 1, len(data)))  # 1 = input
        self.raw_log.write(data)
        self.raw_log.flush()
      self._buffer += data

    pos = self._buffer.find(ETX)
    assert( pos >= 0 )
    data = self._buffer[1:pos]
    self._buffer = self._buffer[pos+1:]
    return data

  def sendCmd( self, cmd ):
    data = STX + cmd + ETX
    self.socket.send(data)
    write_timestamp(self.raw_log)
    self.raw_log.write(struct.pack('HH', 0, len(data)))
    self.raw_log.write(data)
    self.raw_log.flush()
    return self.receive()

  def internalScan( self ):
    time.sleep(0.1)
    data = self.sendCmd( 'sRN LMDscandata' ).split()
    timestamp = time.time()
    remission = None
    if len(data) == 1120:
      dist = [int(x,16) for x in data[26:541+26]]
      remission = [int(x,16) for x in data[541+26+7:-5]]
    elif len(data) == 583:
      dist = [int(x,16) for x in data[26:271+26]]
      remission = [int(x,16) for x in data[304:-5]]
    else:
      dist = [int(x,16) for x in data[26:-6]]
    return timestamp, dist, remission

  def internalScanOld( self ):
    self.sendCmd( 'sMN LMCstartmeas' )
    while 1:
      status = self.queryStatus().split()
      if int(status[2]) >= 7:
        break

    self.sendCmd( 'sEN LMDscandata 1' )
    data = self.receive().split()
#    print data[20:23] # DIST1, scaling, offset
#    print "angleFrom = ", int(data[23],16)
#    print "angleStep = ", int(data[24],16)
#    print "numberData = ", int(data[25],16)
    remission = None
    if len(data) == 1120:
      dist = [int(x,16) for x in data[26:541+26]]
      remission = [int(x,16) for x in data[541+26+7:-5]]
    else:
      dist = [int(x,16) for x in data[26:-6]]
    self.sendCmd( 'sEN LMDscandata 0' )
    return dist, remission

  def startLaser( self ):
    return self.sendCmd( 'sMN LMCstartmeas' )

  def stopLaser( self ):
    self.sendCmd( 'sMN LMCstopmeas' )

  def configureScanDataOutput( self ):
    "BALMS1xxEN_8012471_T763_20090728.pdf, page 100" 
    # better is LMS1xx_LMS5xx_TiM3xx_JEF300_JEF500_English.pdf, page 16
    return self.sendCmd( 'sWN LMDscandatacfg' + 
    " 01 00"+ # output channel
    " 01"+ # output remission values
    " 0"+ # 8bit/16bit
    " 0"+ # unit (default)
    " 00 00"+ # no encoder data
    " 00"+ # output position data
    " 00"+ # output device name
    " 0"+ # output comment
    " 0"+ # output time (maybe we should use true??)
    " +1" # output interval
    )

def name2laser( name ):
  assert( name in ['USB', 'IP'] )
  if name == 'USB':
    return LaserUSB()
  return LaserIP()

def configLaser( name ):
  laser = name2laser( name )
#  print laser.queryScanConfig()
  print laser.configureScanDataOutput()

def testLaser( name, num ):
#  laser = Laser( remission=True )
  laser = name2laser( name )
  laser.start()
  i = 0
  prevScan = None
  while 1:
    scan = laser.scan()
    if scan == prevScan:
      continue
    prevScan = scan
    r = laser.remission()
    print "data", scan
    print "remission", r
#    for s in scan:
#      print "%.2f " % s,
    if scan != None:
      print "%.2f %.2f %.2f" % (min(scan), max(scan), sum(scan)/float(len(scan)))
      if r:
        print max(r)
      i += 1
      if i >= num:
        break
  laser.requestStop()
  laser.join() 


if __name__ == "__main__":
  import sys
  if len(sys.argv) < 3:
    print __doc__
    sys.exit(-1)
  if sys.argv[2] == "--config":
    configLaser( sys.argv[1] )
  else:
    testLaser( sys.argv[1], int(sys.argv[2]) )

 
