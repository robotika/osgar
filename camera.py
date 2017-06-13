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
DEFAULT_URL = "http://192.168.1.6/image?res=half&quality=12&doublescan=0" # half resolution
#DEFAULT_URL = "http://192.168.1.6/image?res=full&x0=0&y0=0&x1=2048&y1=1536&quality=12&doublescan=0&ver=HTTP/1.1" # full resolution ~240kB
DEFAULT_URL_MONO = "http://192.168.1.6/image?channel=mono"
URL_DAY_NIGHT = "http://192.168.1.6/set?daynight=dual" # auto|day|night|dual


# move this to some "common utility"
def timeName( prefix, ext, index = None ):
  today = datetime.date.today()
  t = time.localtime()[3:6]
  suffix = "." + ext
  if index != None:
    suffix = "_%03d" % (index) + suffix
  return prefix + "%02d%02d%02d_%02d%02d%02d" % ( today.year % 100, today.month, today.day, t[0], t[1], t[2] ) + suffix


class Camera( Thread ):
  def __init__(self, verbose = 1, url = DEFAULT_URL, sleep = None ):
    Thread.__init__(self)
    self.setDaemon(True)
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
    self.sleep = sleep

  def getPicture( self, filename, color=True ):
    try:
      if color:
        url = urllib2.urlopen( self.url )
      else:
        url = urllib2.urlopen( DEFAULT_URL_MONO )
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
    # enable also mono camera view
    print urllib2.urlopen( URL_DAY_NIGHT ).read()

    while self.shouldIRun.isSet():
      # first read B&W picture (just for reference, not returned to the "system")
      filename = timeName( "logs/camono", "jpg", index = self._index )
      result = self.getPicture( filename, color=False )
      # result is currently ignored
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


if __name__ == "__main__": 
  cam = Camera( verbose = 2, sleep=0.2 )
  cam.start()
  for i in xrange(10):
    print cam.lastResultEx()
    time.sleep(1)
  cam.requestStop()
  cam.join()

