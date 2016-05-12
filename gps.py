#!/usr/bin/python
"""
  Utility for access GPS data (with experiment with SiRF protocol)

  usage:
      ./gps.py --record | <dir with NMEA> [<sat index>|-1 for all]
"""

import serial
import sys
import os

def checksum( s ):
  sum = 0
  for ch in s:
    sum ^= ord(ch)
  return "%02X" % (sum)

def readNMEA(com):
  while 1:
    ch = com.read(1)
    while ch != "$":
      ch = com.read(1)
    str = ch
    while ch != chr(13):
      ch = com.read(1)
      str += ch 

    assert( str[0] == '$' )
    if str[-4] == '*': # used to be assert
      sum = checksum(str[1:-4])
      if str[-3:-1] != sum:
        print "GPS: bad checksum", str, sum
      if str[-3:-1] == sum:
        return str

def sendNMEACommand( com, cmd ):
  assert( len(cmd) > 0 and cmd[0] == "$" )
  com.write( cmd + "*" + checksum(cmd[1:])+chr(13)+chr(10) )

def sendEnableVTG( com ):
  sendNMEACommand( com, "$PSRF103,05,00,01,01" )

def sendDisableVTG( com ):
  sendNMEACommand( com, "$PSRF103,05,00,00,01" )

def sendSwitchToSiRF( com ):
  sendNMEACommand( com, "$PSRF100,0,4800,8,1,0" )

def ddmm2ddd( s ):
  num,frac = ('0000' + s).split('.')
  d = float(num[-2:]+'.'+frac)/60.0 + float(num[-4:-2]) 
  return d


def parseRMC( nmea ):
  a = nmea.split(",")
  # '$GPRMC', '162111.0', 'A', '5219.9200', 'N', '00951.8819', 'E',
  if a[0] == '$GPRMC' and a[1] and a[2] and a[3] and a[4] and a[5] and a[6]:
    assert( a[4] == 'N' or a[4] == 'S' )
    assert( a[6] == 'E' or a[6] == 'W' )
    return ddmm2ddd( a[3] ), ddmm2ddd( a[5] )
  return None

def parseGGA( nmea ):
  a = nmea.split(",")
  # $GPGGA,051403.000,5005.0319,N,01430.3362,E,1,05,2.5,236.2,M,45.4,M,,0000*54
  if a[0] == '$GPGGA' and a[1]:
    if a[2] and a[3] and a[4] and a[5] and a[6] and a[7] and a[8]:
      # 1 = GPS (0 = invalid)
      # 05 = sats
      # 2.5 = Horizontal Dilution of Precision
      assert( a[3] == 'N' or a[3] == 'S' )
      assert( a[5] == 'E' or a[5] == 'W' )
      return ddmm2ddd( a[2] ), ddmm2ddd( a[4] ), int(a[7]), float(a[8]), float(a[1])
    elif a[7]: # satelites
      return None, None, int(a[7]), None, float(a[1])
  return None

def parseGSV( nmea ):
  a = nmea.split("*")[0].split(",")
  # $GPGSV,3, 1, 11,15,66,284, 40,26,65,296, 34,28,57,067, 31,27,37,275, 21 *71
  # $GPGSV,3, 2, 11,08,30,076, 31,09,22,272, 23,17,21,129, 28,18,20,314, 27 *72
  # $GPGSV,3, 3, 11,05,16,203, 32,24,01,330, 19,19,01,016, *4A
  # 1    = Total number of messages of this type in this cycle
  # 2    = Message number
  # 3    = Total number of SVs in view
  # 4    = SV PRN number
  # 5    = Elevation in degrees, 90 maximum
  # 6    = Azimuth, degrees from true north, 000 to 359
  # 7    = SNR, 00-99 dB (null when not tracking)
  # 8-11 = Information about second SV, same as field 4-7
  # 12-15= Information about third SV, same as field 4-7
  # 16-19= Information about fourth SV, same as field 4-7
  
  if a[0] == '$GPGSV':
    sat = {}
#    print nmea
    for i in xrange(4,len(a)-1, 4):
      if a[i+3]:
        sat[ int(a[i]) ] = (int(a[i+1]), int(a[i+2]), int(a[i+3]))
    return sat

def parsePTNL( nmea ):
  # $PTNL,PJK,154851.00,061714,+5746412.615,N,+686283.416,E,2,05,5.8,EHT+123.804,M*45
  a = nmea.split("*")[0].split(",")
  if a[0] == '$PTNL':
    return a

#-------------------------------------------
def checksumSiRF( data ):
  sum = 0
  for x in data:
    sum += x
  return sum & 0x7FFF

def sendSiRFCommand( com, cmd ):
  sum = checksumSiRF( cmd )
  data = [ 0xA0, 0xA2, len(cmd)>>8, len(cmd) & 0xFF ] + cmd \
     + [ sum>>8, sum & 0xFF, 0xB0, 0xB3 ];
  packet = "".join( [ chr(x) for x in data ] ) 
  com.write( packet )

def sendSiRFSwitchToNMEA( com ):
  sendSiRFCommand( com, [ 0x81, 0x02, 1, 1, 0, 1, 1, 1, 5, 1, 1, 1, 0, 1, \
    0, 1, 0, 1, 0, 1, 0, 1, 0x12, 0xC0 ] )

def readSiRF( com ):
#  head1 = com.read(1)
#  while head! != 0xA0:
#    pass

  data = com.read(8)
  return [ hex( ord(x) ) for x in data ]


from threading import Thread,Event,Lock
import time
import datetime

def timeName( prefix, ext ):
  today = datetime.date.today()
  t = time.localtime()[3:6]
  return prefix + "%02d%02d%02d_%02d%02d%02d" % ( today.year % 100, today.month, today.day, t[0], t[1], t[2] ) + "." + ext


class GPS( Thread ):
  def __init__(self, verbose = 1):
    Thread.__init__(self)
    self.setDaemon(True)
    self.verbose = verbose
    self.lock = Lock()
    self.shouldIRun = Event()
    self.shouldIRun.set()
    self._coord = None
    self._satelites = -1 # unknown
    filename = timeName( "logs/gps", "nmea" )
    print filename
    self._logFile = open( filename, "wb" )
    if os.name == 'nt': # windows (could be also used sys.platform == 'win32'
      self.com = serial.Serial( 'COM4', 4800 ) # sometimes COM9 on MD laptop
    else:
      self.com = serial.Serial( '/dev/ttyUSB0', 4800 )

  def run(self):
    while self.shouldIRun.isSet():
      nmea = readNMEA( self.com ).strip()
      self._logFile.write( nmea + "\r\n" )
      self._logFile.flush()
      if self.verbose:
        print nmea
      tmpCoord = parseGGA( nmea )
      if tmpCoord:
        self._coord = tmpCoord
        if self.verbose:
          print self._coord

  def coord(self):
    self.lock.acquire()
    xy = self._coord
    self.lock.release()
    return xy

  def requestStop(self):
    self.shouldIRun.clear()


class DummyGPS():
  def start( self ):
    pass

  def coord( self ):
    return 0.0, 0.0, 4, 1.0, 1.0

  def requestStop( self ):
    pass


def usage():
  print __doc__

if __name__ == "__main__": 

  if len(sys.argv) < 2:
    print __doc__
    sys.exit(2)

  if sys.argv[1] != "--record":
    import os
#    print "Parsing NMEA"
#    for filename in sys.argv[1:]:
    dir = sys.argv[1]
    satIndex = None
    if len(sys.argv) > 2:
      satIndex = int(sys.argv[2])
    names = os.listdir( dir )
    i = 0
    for filename in names:
      if ".nmea" in filename:
        printPosition = True #False
#        print filename
        file = open( dir + "/" + filename, "rb" )
        line = file.readline()
        while line:
#          pos = parseRMC( line )
          pos = parseGGA( line )
          if pos and printPosition:
            print filename+";"+str(i)+";"+str(pos[0])+";"+str(pos[1])
#            print pos
            i += 1
          sat = parseGSV( line )
          if sat:
            if satIndex and satIndex in sat:
              print sat[satIndex]
            elif satIndex == -1:
              print sat
          line = file.readline()
        file.close()
    sys.exit(0)

  gps = GPS()
  gps.start()
  print gps.coord()
  time.sleep(2)
  gps.requestStop()
  gps.join()

