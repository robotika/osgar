#!/usr/bin/python
"""
  IP camera utility with image processing

  usage:
      ./camera.py
"""

from threading import Thread,Event,Lock
import time
import datetime
import urllib2
import subprocess
import select
import socket
import sys
import os

# Default camera URL
DEFAULT_URL = "http://192.168.1.6/img.jpg"

# move this to some "common utility"
def timeName( prefix, ext, index = None ):
  today = datetime.date.today()
  t = time.localtime()[3:6]
  suffix = "." + ext
  if index != None:
    suffix = "_%03d" % (index) + suffix
  return prefix + "%02d%02d%02d_%02d%02d%02d" % ( today.year % 100, today.month, today.day, t[0], t[1], t[2] ) + suffix


camYTab = (
      ( 512.0, 0.15 ),
      ( 452.0, 0.25 ),
      ( 390.0, 0.35 ),
      ( 341.0, 0.45 ),
      ( 306.0, 0.55 ),
      ( 279.0, 0.65 ),
      ( 258.0, 0.75 ),
      ( 243.0, 0.85 ),
      ( 0.0, 2.0 )
      )

camXTab = (
    ( 640.0, -0.6 ),
    ( 533.0, -0.5 ),
    ( 513.0, -0.4 ),
    ( 478.0, -0.3 ),
    ( 437.0, -0.2 ),
    ( 389.0, -0.1 ),
    ( 338.0, 0 ),
    ( 287.0, 0.1 ),
    ( 239.0, 0.2 ),
    ( 198.0, 0.3 ),
    ( 163.0, 0.4 ),
    ( 143.0, 0.5 ),
    ( 36.0, 0.6 ),
    ( 0.0, 0.7 ) )

def interpolate( val, tab ):
  for i in range(len(tab)):
    if val > tab[i][0]:
      break
  assert( i > 0 )
  return tab[i-1][1] + (val - tab[i-1][0])/(tab[i][0]-tab[i-1][0]) * (tab[i][1]-tab[i-1][1])

def img2xy( (x,y) ):
  "convert image coordinates into robot relative coordinates"
  front = interpolate( y, camYTab)
  # (front = 0.55 -> scale 1.0
  return front, interpolate( x, camXTab )   #*(1+1.3919597989949748*(0.55-front))


class ImageProc():
  def __init__(self, exe = "./king", verbose = 1, priority = None):
    self.verbose = verbose
    priority_fn = None if priority is None or not hasattr(os, 'nice') else lambda : os.nice(priority)
    self.process = subprocess.Popen( exe, shell=True, 
        stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, preexec_fn = priority_fn)
    # Win problem: not supported close_fds=True ... is it critical?
    # note, that popen2/3 handled incorrect assignment of stdin/stdout

  def processPicture( self, filename ):
    if self.verbose:
      if self.verbose == 1:
        sys.stderr.write('o')
      else:
        print "process", filename
#    self._fout.write( "set roi 0 256 640 256\n" )
#    self._fout.write( "set roi 0 128 640 256\n" )

    self.process.stdin.write( "file "+filename+"\n" )
    self.process.stdin.flush()
    if select.select([self.process.stderr],[],[],0)[0]: # Win problem, and not clear how much I can read?
      line = self.process.stderr.readline()
      if not line.startswith( "Corrupt JPEG data:" ):
        sys.stderr.write( line )
    return self.process.stdout.readline()

class DummyProc():
  def __init__( self, sleep = None ):
    self.sleep = sleep
  def processPicture( self, filename ):
    if self.sleep != None:
      time.sleep( self.sleep )

class Camera( Thread ):
  def __init__(self, imageProc = None, verbose = 1, url = DEFAULT_URL, sleep = None ):
    Thread.__init__(self)
    self.setDaemon(True)
    self.imageProc = imageProc
    self.verbose = verbose
    self.lock = Lock()
    self.shouldIRun = Event()
    self.shouldIRun.set()
    filename = timeName( "logs/cam", "txt" )
    print filename
    self._logFile = open( filename, "wb" )
    self._lastResult = None
    self._lastResultFile = None
    self._index = 0
    self.url = url
    self.queryCount = 0
    self.sleepProc = DummyProc( sleep )
    self.sleep = sleep
    self.pausedProcessing = False
    self.snapshotOnly = False

  def pauseImageProcessing( self, pause ):
    self.lock.acquire()
    self.pausedProcessing = pause
    self.lock.release()

  def processNextSnapshot( self ):
    self.lock.acquire()
    self.pausedProcessing = False
    self.snapshotOnly = True
    self.lock.release()

  def getPicture( self, filename ):
    try:
      url = urllib2.urlopen( self.url )
      img = url.read()
      t = time.time()
      file = open( filename, "wb" )
      file.write( img )
      file.close()
      return t, img
    except IOError, e:
      print e
      return None

  def run(self):
    while self.shouldIRun.isSet():
      filename = timeName( "logs/cam", "jpg", index = self._index )
      self._index += 1
      if self.verbose:
        if self.verbose == 1:
          sys.stderr.write('.')
        else:
          print "Getting picture", filename
      result = self.getPicture( filename )
      if result is not None:
        at, __ = result
        if self.pausedProcessing:
          imageProc = self.sleepProc
        else:
          imageProc = self.imageProc
        self.lock.acquire()
        if self.snapshotOnly:
          self.snapshotOnly = False
          self.pausedProcessing = True
        self.lock.release()
        if imageProc:
          tmpResult = imageProc.processPicture( filename )
        else:
          tmpResult = filename, None
        self.lock.acquire()
        self._logFile.write( str(self.queryCount) + '\t' + str(at) + "\n" )
        self.queryCount = 0
        self._lastResult = tmpResult
        self._lastResultFile = filename
        self.lock.release()
        self._logFile.write( filename + "\t" + str(self._lastResult) )
        self._logFile.flush()
        if self.verbose>1:
          print "Camera:", self._lastResult
        if self.sleep:
          time.sleep(self.sleep)
      else:
        # read picture failed
        time.sleep(1.0)

  def lastResult(self):
    self.lock.acquire()
    ret = self._lastResult
    self.lock.release()
    return ret

  def lastResultEx(self):
    self.lock.acquire()
    ret = (self._lastResult, self._lastResultFile)
    self.queryCount += 1
    self.lock.release()
    return ret

  def requestStop(self):
    self.shouldIRun.clear()


class CameraFromLog:
  def __init__( self, filename ):
    self.genLines = open( filename )
    self.countQuery = int( self.genLines.next() )
    self.lastResult = (None,None)

  def start( self ):
    pass

  def requestStop( self ):
    pass

  def lastResultEx( self ):
    if self.countQuery > 0:
      self.countQuery -= 1
      return self.lastResult
    line = self.genLines.next()
    self.lastResult = (" ".join( line.split()[1:]) + "\n", line.split()[0] )
    self.countQuery = int( self.genLines.next() ) - 1 # this result is reported now -> -1
    return self.lastResult

class RemoteCamera(Thread):
  def __init__(self, address, verbose = 0):
    Thread.__init__(self)
    self.setDaemon(True)

    self.verbose = verbose

    self.skt = socket.socket()
    self.skt.connect(address)

    # Send even small packets, do not become late because
    # of the Naggle's algorithm
    self.skt.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    self.io = self.skt.makefile()

    self.lock = Lock()
    self.__lastResult = None
    self.__lastResultFile = None
    self.shouldIRun = Event()

  def sendCmd(self, cmd):
    if cmd[-1] != '\n':
      cmd += '\n'

    self.io.write(cmd)
    self.io.flush()

  def run(self):
    self.sendCmd('go')

    self.shouldIRun.set()

    while self.shouldIRun.isSet():
      imgname = self.io.readline()
      result = self.io.readline()
      self.lock.acquire()
      self.__lastResult = result
      self.__lastResultFile = imgname
      self.lock.release()

  def lastResult(self):
    self.lock.acquire()
    ret = self.__lastResult
    self.lock.release()
    return ret

  def lastResultEx(self):
    self.lock.acquire()
    ret = (self.__lastResult, self.__lastResultFile)
    self.lock.release()
    return ret

  def requestStop(self):
    self.sendCmd('stop')
    self.shouldIRun.clear()


class DummyCamera():
  def start( self ):
    pass

  def requestStop( self ):
    pass

  def pauseImageProcessing( self, pause ):
    pass

  def processNextSnapshot( self ):
    pass

def usage():
  print __doc__

if __name__ == "__main__": 

#  if len(sys.argv) < 2:
#    usage()
#    sys.exit(2)

  cam = Camera( verbose = 2, sleep=0.2 )
  #cam = RemoteCamera(('', 8431))
  cam.start()
  for i in xrange(10):
    print cam.lastResultEx()
    time.sleep(1)
  cam.requestStop()
  cam.join()

