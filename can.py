#!/usr/bin/python
"""
  CAN bus interface over serial port.
  usage:
    can.py [<file to parse> | [modules to watch (default all)]]
"""

import serial
import datetime
import time
import os
import os.path
import socket
import subprocess
import sys

class LogEnd(Exception):
  pass

class LogIt():
  "Log communication via serial com"
  def __init__( self, com ):
    self._com = com
    self._logFile = None
    self.relog( 'logs/b' )

  def relog( self, prefix ):
    today = datetime.date.today()
    t = time.localtime()[3:6]
    filename = prefix + "%02d%02d%02d_%02d%02d%02d.log" % ( today.year % 100, today.month, today.day, t[0], t[1], t[2] )
    self._logFile = open( filename, "wb" )
    print "LogIt:", filename
    return filename

  def read( self, numChars ):
    s = self._com.read( numChars )
    for ch in s:
      self._logFile.write( chr(0x01) )
      self._logFile.write( ch )
    self._logFile.flush()
    return s

  def write( self, chars ):
    for ch in chars:
      self._logFile.write( chr(0x00) )
      self._logFile.write( ch )
    self._logFile.flush()
    self._com.write( chars )

#-------------------------------------------------------------------

class ReplyLog():
  "Read & verify log"
  def __init__( self, filename, assertWrite=True ):
    print "ReplyLog", filename
    self._logFile = open( filename, "rb" )
    self.assertWrite = assertWrite

  def read( self, numChars ):
    s = []
    for i in range(numChars):
      marker = self._logFile.read(1)
      if not marker:
        raise LogEnd()
      assert( marker == chr(0x01) )
      s.append(self._logFile.read(1))
      if not s[-1]:
        raise LogEnd()
    return ''.join(s)

  def write( self, chars ):
    for ch in chars:
      marker = self._logFile.read(1)
      if not marker:
        raise LogEnd()
      assert( marker == chr(0x00) )
      verifyCh = self._logFile.read(1)
      if not verifyCh:
        raise LogEnd()
      if self.assertWrite:
        assert( verifyCh == ch )

#-------------------------------------------------------------------

class ReplyLogInputsOnly():
  "Read & verify log"
  def __init__( self, filename ):
    print "ReplyLogInputOnly", filename
    self._logFile = open( filename, "rb" )

  def read( self, numChars ):
    s = []
    for i in range(numChars):
      while( self._logFile.read(1) not in [chr(0x01), ''] ):
        c = self._logFile.read(1) # skip write output
        if not c:
          raise LogEnd()
        assert( i == 0 ) # the packets should be complete
      s.append(self._logFile.read(1))
      if not s[-1]:
        raise LogEnd()
    return ''.join(s)

  def write( self, chars ):
    pass

#-------------------------------------------------------------------

class DummyMemoryLog():
  "Helper for parsing logs"
  def __init__( self ):
    self.data = 10*[0xFF]   # workaround for initial sync

  def read( self, numChars ):
    s = ""
    if self.data:
      s = "".join( [chr(x) for x in self.data[:numChars]] )
      self.data = self.data[numChars:]
    return s

  def write( self, chars ):
    pass

#-------------------------------------------------------------------

class RTSerial():
  "Handle i/o via real-time serial pipe"
  # An OSError (32: Broken Pipe) occures when the real-time serial
  # process (rt42) has died. This process dies if we stop reading data
  # from it for a while (eg. during a restart) and the communication
  # buffer gets full. => the communication may need a restart,
  # which this class performs in such a case.
  def __init__( self, com ):
    self.sub = None
    self.com = com
    self.__connect()

  def __connect( self ):
    sys.stderr.write("RTSerial - connect ...\n")
    self.__disconnect()
    self.sub = subprocess.Popen(["/home/robot/system/rt42"])
    time.sleep(0.1)
    self._fd = os.open( self.com, os.O_RDWR )

  def __disconnect( self, sleep = False ):
    if self.sub:
#      os.close( self._fd )
      if sleep:
        time.sleep(0.1)
      # self.sub.kill() # not available in Python 2.5
      #killCmd = ["kill", "-9", "%d" % self.sub.pid]
      pid = self.sub.pid
      del self.sub
      self.sub = None
      os.close( self._fd )
      del self._fd
      self._fd = None
      time.sleep(1.0)
      killCmd = ["kill", "-9", "%d" % pid]
      print killCmd
      subprocess.call( killCmd )
      time.sleep(1.0)
      print "killed"

  def __del__( self ):
    self.__disconnect(sleep=True)

  def read( self, numChars ):
    if self.sub and self.sub.returncode:
      print self.sub.returncode
    s = []
    n = 0
    while n < numChars:
      try:
        rec = os.read( self._fd, numChars - len(s) )
        s.append(rec)
        n += len(rec)
      except OSError: # Broken Pipe
        self.__connect()
    return ''.join(s)

  def write( self, chars ):
    try:
      os.write( self._fd, chars )
    except OSError: # Broken pipe
      self.__connect()

#-------------------------------------------------------------------

class CAN():
  def __init__(self, com = None, defaultDTR = False, verbose = 1, skipInit = False):
    self.verbose = verbose
    self.skipInit = skipInit
    
    if com:
      useDTR = defaultDTR
      self.com = com
    else:
      if os.name == 'nt': # windows (could be also used sys.platform == 'win32'
        ser = serial.Serial('COM4',115200,dsrdtr=0) # alt COM9
        useDTR = True
      else:
        ser = RTSerial('/dev/rtp0')                 # alt /dev/ttyS0
        useDTR = False
      if useDTR:
        ser.setDTR(0)
      self.com = LogIt( ser )

    if useDTR:
      assert( self.readPacket() == (0xFE,[0x10]) ) # wait for BootUp message

    if not skipInit:
      self.syncFF()
      self.sendControl( 0x31 ) # start bridge

  def __del__(self):
    if not self.skipInit:
      self.sendControl( 0x30 ) # stop bridge
    del self.com
    print "CAN terminated"
  
  def syncFF(self):
    "synchronize CANBridge communication"
    self.com.write(chr(0xff)*10)
    i = 0
    while i < 10:
      s = self.com.read(1)
      if ord(s[0]) == 0xFF:
        i += 1
      else:
        i = 0
    if self.verbose > 0:
      print "syncFF OK"

  def sendControl(self, command):
    self.com.write(chr(0xfe)+chr(command))

  def readPacket(self):
    b = 0xFF
    while b == 0xFF:
      b = ord( self.com.read( 1 ) )
    header = [b]
    header.append( ord( self.com.read( 1 ) ) )
    rtr=(header[1]>>4)&0x1
    len=(header[1])&0x0f
    id=((header[0])<<3)|(((header[1])>>5)&0x1f)
    if rtr:
      if header[0]==0xfe:
        if header[1] == 0x10:
          print "BRIDGE: boot-up message"
        elif header[1] == 0x30:
          print "BRIDGE: connected to CAN, error active"
        else:
          print "Error:%x%x"%(header[0],header[1])
        return header[0], [header[1]]
      else:
        print "RTR:", id, "len=", len
        return id,[len]
    else:
      #print "id ",id,"len",len,"rtr ",rtr
      data=[ord(x) for x in self.com.read(len)]
      return id,data

  def sendData(self,id,data):
    header=[(id>>3)&0xff,(id<<5)&0xe0|(len(data)&0xf)]
    packet="".join([chr(x) for x in header+data])
    self.com.write(packet)

  def printPacket( self, id, data ):
    print hex(id), ":", data

  def sendOperationMode( self ):
    self.sendData(0,[1,0])

  def sendPreoperationMode( self ):
    self.sendData(0,[128,0])
    
  def collectPeriodPackets( self ):
    packets = {}
    id = 0
    while id!=128:
      id,data=self.readPacket()
      packets[ id ] = data
    return packets

  def relog( self, prefix ):
    if "relog" in dir(self.com):
      return self.com.relog( prefix )

  def resetModules( self, configFn=None ):
    print "Reset all modules"
    self.sendData( 0, [129,0] ) # reset all

    ackBootup = [] # Heart Beat 0, bootup, after reset
    ackPreop = [] # HB 127
    ackOp = [] # HB 5

    while len( ackBootup ) == 0 or len( ackBootup ) > len( ackPreop ):
      id, data = self.readPacket()
      if (id & 0xF80) == 0x700:
        nodeID = id & 0x7F
        if data[0] == 0:
          print "Started module", nodeID
          ackBootup.append( nodeID )
        if data[0] == 127:
          if nodeID in ackBootup:
            print "Module", nodeID, "in preoperation."
            ackPreop.append( nodeID )
          else:
            print "WARNING!!! Module", nodeID, "preop BEFORE bootup!!!"

    if configFn:
      print "--- Extra Configuration ---"
      configFn( self )

    print "------- Switch to Operation mode --------"
    self.sendOperationMode()
    while len( ackPreop ) > len( ackOp ):
      id, data = self.readPacket()
      if (id & 0xF80) == 0x700:
        nodeID = id & 0x7F
        if data[0] == 5:
          if nodeID in ackPreop:
            print "Module", nodeID, "in operation."
            ackOp.append( nodeID )
          else:
            print "WARNING!!! Module", nodeID, "op BEFORE preop!!!"

    print "collecting some packets ..."
    countHB = 0
    while countHB < len( ackOp) * 3: # ie approx 3s
      id, data = self.readPacket()
      if (id & 0xF80) == 0x700:
        countHB += 1
        if countHB % len( ackOp ) == 0:
          print countHB/len( ackOp ), '...'
        assert( len(data) == 1 )
        if data[0] != 5:
          nodeID = id & 0x7F
          print 'ERROR - module', nodeID, 'data', data

    return ackOp

def main():
  can = CAN()
  can.sendOperationMode()
  print can.collectPeriodPackets()
  print can.collectPeriodPackets()
  print can.collectPeriodPackets()
  can.sendPreoperationMode()


def parseFileG( filename, watchModules = [] ):
  # dump log file
  dummy = DummyMemoryLog()
  can = CAN( com=dummy )
  file = open( filename, "rb" )
  d = []
  prevIo = 2
  binData = [ord(x) for x in file.read()] + [2,0] # dummy termination
  pairs = zip( binData[::2], binData[1::2] )
  for io, data in pairs:
    if io == prevIo:
      d.append( data )
    else:
      if prevIo == 0:
        # parse send
        while d:
          assert( len(d) >= 2 )
          if d[0] == 0xFE:
            print "OutFE:", hex(d[0]), hex(d[1])
            dataSize = 0
          else:
            id = (int(d[0])<<3) | (d[1]>>5)
            dataSize = d[1] & 0x0F
            if not watchModules or (id & 0x7F) in watchModules:
              yield 0, id, d[2:2+dataSize]
          d = d[2+dataSize:]
      elif prevIo == 1:
        assert( len(d) >= 2 )
        assert( dummy.data == [] )
        dummy.data = d
        while dummy.data:          
          if dummy.data == 10*[0xFF]:
            # skip syncFF without new packets
            dummy.data = []
            break
          a,b = can.readPacket()
          if not watchModules or (a&0x7F) in watchModules:
            yield 1, a, b

      d = [data]
    prevIo = io
  file.close()

def parseFile( filename, watchModules = [] ):
  for io, id, data in parseFileG( filename, watchModules ):
    if io == 0:
      print "------------"
      print "Out:", hex(id), data
    elif io == 1:
      print hex(id), ":", data


if __name__ == "__main__":
  import sys
  if len(sys.argv) > 1:
    parseFile( sys.argv[1], [int(x) for x in sys.argv[2:]] )
  else:
    main()

