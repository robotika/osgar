#!/usr/bin/python
"""
  Utility for read/write Service Data Objects

  usage:
      ./sdoplg.py <nodeID:index:subindex> [-wfsb] [<filename>]

      -w ... write
      -f ... file
      -s ... string
      -b ... byte
      -h ... hex
"""

from can import CAN
import sys

def printAbort( errClass, errCode, addCode ):
  "Print About message for given class, code and additional code"
  errClassMsg = { 
     5:"Service Error", 
     6:"Access Error", 
     8:"Other Error"
      }
  errCodeMsg = { 
      1:"Object access unsupported", 
      2:"Object non-existent", 
      3:"Parameter Inconsistent",
      6:"Hardware fault",
      7:"Type conflict",
      }
  errTri = {
      (5,3,0):"Toggle bit not alternated",
      (6,2,0):"Object does not exist",
      (6,6,0):"Acceess failed due to hardware",
      (6,7,0x12):"Object length too high",
      (6,7,0x13):"Object length too low"
      }
  try:
    print "Abort:", errTri[  (errClass, errCode, addCode ) ] + " (" + errClassMsg[ errClass ] + ", " + errCodeMsg[ errCode ] + ")"
  except:
    print "Unknown abort code:", (errClass, errCode, addCode )


class SDOPlg:
  def __init__(self, can = None, verbose = 1):
    if can is None:
      can = CAN()
#      can.resetModules() # event this sometimes does not help :(
    self.can = can
    self.verbose = verbose
  
  def readSDO( self, nodeID, objectIndex, subIndex ):
    "Read Dictionary Object via CANopen Service Data Objects (SDO)"
    self.can.sendData( 0x600 + nodeID, [0x40, objectIndex&0xFF, objectIndex>>8, subIndex, 0,0,0,0] )
    toggleBit = 0x00
    wholeData = []
    while True:
      id,data = self.can.readPacket()
      self.can.printPacket( id, data )
      if id == 0x580 + nodeID:
        assert( len(data) == 8 )
        if data[0] == 0x80: # Abort
          printAbort( data[7], data[6], (data[5]<<8)|data[4] ) 
          return [] # throw exception?

        if (data[0] & 0xF3) == 0x43: # expedited transfer
          assert( ((data[2]<<8)|data[1]) == objectIndex )
          assert( data[3] == subIndex )
          num = 4 - ((data[0]>>2) & 0x3)
          return data[4:4+num]

        # multi packet transfer (segmented)
        if data[0] == 0x41 : # init
          assert( ((data[2]<<8)|data[1]) == objectIndex )
          assert( data[3] == subIndex )
          totalLength = data[4] + (data[5]<<8) + (data[6]<<16) + (data[7]<<24)
          print "TotalLength", totalLength
        else:
          assert( (data[0] & 0xF0) == 0x00 | toggleBit )
          toggleBit = 0x10 - toggleBit
          if data[0] & 0x01: # last segment
            num = 7 - ((data[0]>>1) & 0x7 )
            wholeData.extend( data[1:1+num] )
            return wholeData
          else:
            wholeData.extend( data[1:] )
        self.can.sendData( 0x600 + nodeID, [0x60|toggleBit, objectIndex&0xFF, objectIndex>>8, subIndex, 0,0,0,0] )

  def writeSDO( self, nodeID, objectIndex, subIndex, data ):
    "Write Dictionary Object via CANopen Service Data Objects (SDO)"
    remainingData = []
    if len(data) <= 4:
      # expedited transfer
      cmd = 0x23 | ((4-len(data))<<2)
      d = data
      d.extend( [0,0,0,0] )
      self.can.sendData( 0x600 + nodeID, [cmd, objectIndex&0xFF, objectIndex>>8, subIndex, d[0],d[1],d[2],d[3]] )
    else:
      cmd = 0x21
      l = len(data)
      self.can.sendData( 0x600 + nodeID, [cmd, objectIndex&0xFF, objectIndex>>8, subIndex, l&0xFF, (l>>8)&0xFF, (l>>16)&0xFF, (l>>24)&0xFF] )
      remainingData = data

    toggleBit = 0x00
    while True:
      id,data=self.can.readPacket()
      self.can.printPacket( id, data )
      if id == 0x580 + nodeID:
        assert( len(data) == 8 )
        if data[0] == 0x80: # Abort
          printAbort( data[7], data[6], (data[5]<<8)|data[4] ) 
          return # throw exception?

        sendNextData = False
        if data[0] == 0x60: # both expedited & segmented transfer
          assert( ((data[2]<<8)|data[1]) == objectIndex )
          assert( data[3] == subIndex )
          sendNextData = True

        if data[0] == 0x20 or data[0] == 0x30:
          assert( data[0] & 0x10 == toggleBit )
          toggleBit = 0x10 - toggleBit
          sendNextData = True

        if sendNextData:
          if len(remainingData) == 0:
            return
          # next 7 bytes
          if len(remainingData) > 7:
            cmd = 0x00 | toggleBit
            self.can.sendData( 0x600 + nodeID, [cmd] + remainingData[0:7] )
            remainingData = remainingData[7:]
          else:
            cmd = 0x01 | toggleBit | ((7-len(remainingData))<<1) # set last segment
            d = remainingData
            d.extend( [0,0,0,0,0,0,0] )
            self.can.sendData( 0x600 + nodeID, [cmd] + d[0:7] )
            remainingData = []

#------------------------------------------------------------------------------
class ReadSDO:
  "Read Dictionary Object via CANopen Service Data Objects (SDO) - iteraction implemenation"
  def __init__( self, nodeID, objectIndex, subIndex ):
    self.nodeID, self.objectIndex, self.subIndex = nodeID, objectIndex, subIndex
    self.result = None
    self.packet = None

  def update( self, packet ):
    "update data from can - buffer for 1 packet only!"
    self.packet = packet
    
  def generator( self ):
    yield ( 0x600 + self.nodeID, [0x40, self.objectIndex&0xFF, self.objectIndex>>8, self.subIndex, 0,0,0,0] )
    toggleBit = 0x00
    wholeData = []
    while True:
      while self.packet is None:
        yield None
      id,data = self.packet
      self.packet = None
      if id == 0x580 + self.nodeID:
        assert( len(data) == 8 )
        if data[0] == 0x80: # Abort
          printAbort( data[7], data[6], (data[5]<<8)|data[4] ) 
          self.result = [] # throw exception?
          return

        if (data[0] & 0xF3) == 0x43: # expedited transfer
          assert( ((data[2]<<8)|data[1]) == self.objectIndex )
          assert( data[3] == self.subIndex )
          num = 4 - ((data[0]>>2) & 0x3)
          self.result = data[4:4+num]
          return

        # multi packet transfer (segmented)
        if data[0] == 0x41 : # init
          assert( ((data[2]<<8)|data[1]) == self.objectIndex )
          assert( data[3] == self.subIndex )
          totalLength = data[4] + (data[5]<<8) + (data[6]<<16) + (data[7]<<24)
        else:
          assert( (data[0] & 0xF0) == 0x00 | toggleBit )
          toggleBit = 0x10 - toggleBit
          if data[0] & 0x01: # last segment
            num = 7 - ((data[0]>>1) & 0x7 )
            wholeData.extend( data[1:1+num] )
            self.result = wholeData
            return
          else:
            wholeData.extend( data[1:] )
        yield ( 0x600 + self.nodeID, [0x60|toggleBit, self.objectIndex&0xFF, self.objectIndex>>8, self.subIndex, 0,0,0,0] )



class WriteSDO:
  "Write Dictionary Object via CANopen Service Data Objects (SDO)"
  def __init__( self, nodeID, objectIndex, subIndex, data ):
    self.nodeID, self.objectIndex, self.subIndex = nodeID, objectIndex, subIndex
    self.data = data

  def update( self, packet ):
    "update data from can - buffer for 1 packet only!"
    self.packet = packet

  def generator( self ):
    remainingData = []
    if len(self.data) <= 4:
      # expedited transfer
      cmd = 0x23 | ((4-len(self.data))<<2)
      d = self.data
      d.extend( [0,0,0,0] )
      yield ( 0x600 + self.nodeID, [cmd, self.objectIndex&0xFF, self.objectIndex>>8, self.subIndex, d[0],d[1],d[2],d[3]] )
    else:
      cmd = 0x21
      l = len(self.data)
      yield ( 0x600 + self.nodeID, [cmd, self.objectIndex&0xFF, self.objectIndex>>8, self.subIndex, l&0xFF, (l>>8)&0xFF, (l>>16)&0xFF, (l>>24)&0xFF] )
      remainingData = self.data

    toggleBit = 0x00
    while True:
      while self.packet is None:
        yield None
      id,data = self.packet
      self.packet = None
#      self.can.printPacket( id, data )
      if id == 0x580 + self.nodeID:
        assert( len(data) == 8 )
        if data[0] == 0x80: # Abort
          printAbort( data[7], data[6], (data[5]<<8)|data[4] ) 
          return # throw exception?

        sendNextData = False
        if data[0] == 0x60: # both expedited & segmented transfer
          assert( ((data[2]<<8)|data[1]) == self.objectIndex )
          assert( data[3] == self.subIndex )
          sendNextData = True

        if data[0] == 0x20 or data[0] == 0x30:
          assert( data[0] & 0x10 == toggleBit )
          toggleBit = 0x10 - toggleBit
          sendNextData = True

        if sendNextData:
          if len(remainingData) == 0:
            return
          # next 7 bytes
          if len(remainingData) > 7:
            cmd = 0x00 | toggleBit
            yield ( 0x600 + self.nodeID, [cmd] + remainingData[0:7] )
            remainingData = remainingData[7:]
          else:
            cmd = 0x01 | toggleBit | ((7-len(remainingData))<<1) # set last segment
            d = remainingData
            d.extend( [0,0,0,0,0,0,0] )
            yield ( 0x600 + self.nodeID, [cmd] + d[0:7] )
            remainingData = []


def testNewSDO_0( filename ):
  from can import ReplyLog
  can=CAN(ReplyLog(filename))
  can.resetModules()
  sdoplg = SDOPlg( can )
  ( nodeID, dataIndex, subIndex ) = 10, 16*256+10, 0
  data = sdoplg.readSDO( nodeID, dataIndex, subIndex )
  print "RESUT DATA:", data

def testNewSDO( filename ):
  from can import ReplyLog
  can=CAN(ReplyLog(filename))
  can.resetModules()
  ( nodeID, dataIndex, subIndex ) = 10, 16*256+10, 0
  reader = ReadSDO( nodeID, dataIndex, subIndex )
  for packet in reader.generator():
    if packet != None:
      can.sendData( *packet )
    reader.update( can.readPacket() )
  print "RESUT DATA:", reader.result

#------------------------------------------------------------------------------

def detectI2CModules( com, nodeID ):
  modules = []
  for addr in range(128):
    writeSDO( com, nodeID, 0x2101, 0, [addr] )
    data = readSDO( com, nodeID, 0x2100, 0 )
    if data:
      modules.append( addr )
  return modules

def processSDO( tripple, operation, dataToSend ):
  print tripple, operation, len(dataToSend)

  sdoplg = SDOPlg()
#  print "I2C Modules:", detectI2CModules( com, 7 )

  print "TestSDO", tripple
  nodeID, dataIndex, subIndex = tripple

  if operation == "read":
    data = sdoplg.readSDO( nodeID, dataIndex, subIndex )
    return data

  assert( operation == "write" )
  sdoplg.writeSDO( nodeID, dataIndex, subIndex, dataToSend )

def usage():
  print __doc__

if __name__ == "__main__": 
  if len(sys.argv) < 2:
    usage()
    sys.exit(2)
  assert( len( sys.argv[1].split(":") ) == 3 )
  tripple = tuple( [int(x,0) for x in sys.argv[1].split(":")] )

  options = []
  if len(sys.argv) > 2 and sys.argv[2][0] == "-":
    options = [ x for x in sys.argv[2][1:] ]

  operation = "read"
  if "w" in options:
    operation = "write"
  
  filename = ""
  if "f" in options:
    assert( len(sys.argv) > 2 and sys.argv[-1][0] != "-" )
    filename = sys.argv[-1]


  data = []
  if operation == "write":
    if filename:
      file = open( filename, "rb" )
      str = file.read()
      data = [ord(x) for x in str]
      file.close()
    if "s" in options:
      assert( len(sys.argv) > 3 )
      data = [ ord(x) for x in sys.argv[3] ]
    if "b" in options:
      assert( len(sys.argv) > 3 )
      assert( filename == "" )
      data = [ int(x,0) for x in sys.argv[3:] ]

  assert( operation == "read" or len(data) > 0 )

#  print sys.argv, tripple, operation, filename, options, data
  data = processSDO( tripple, operation, data ) 

  if operation == "read":
    if filename:
      file = open( filename, "wb" )
      file.write( "".join([chr(x) for x in data]) )
      file.close()
    if "s" in options:
      print "".join([chr(x) for x in data])
    elif "h" in options:
      print " ".join([hex(x) for x in data])
    else:
      print data

