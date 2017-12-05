#!/usr/bin/env python
import serial
from threading import Thread

class DummyController:
  def __init__( self ):
    self.status = {}
  def send( self, nodeId, cmd ):
    print("SEND", nodeId, cmd)
    self.status[ nodeId ] = cmd

  def query( self, nodeId ):
    return self.status.get( nodeId, "unknown" )

class Controller( Thread ):
  def __init__( self, comname ):
    self.com = serial.Serial( comname, 9600, timeout=1.0 )
    self.status = {}
    Thread.__init__(self)
    self.setDaemon(True) 
    self.start()

  def send( self, nodeId, cmd ):
    print("SEND", nodeId, cmd)
    self.com.write( str( ("SEND", nodeId, cmd) ) + "\r\n" )
    self.status[ nodeId ] = cmd # pre-optimistics, remove if necessary

  def query( self, nodeId ):
    return self.status.get( nodeId, "unknown" )

  def run( self ):
    print("THREAD")
    while( 1 ):
      s = self.com.read(1)
      print(s)
      if len(s) > 0:
        ch = s[0]
        if ch >= 'a' and ch <= 'z':
          self.status[ ord(ch)-ord('a')+1 ] = "off"
        if ch >= 'A' and ch <= 'Z':
          self.status[ ord(ch)-ord('A')+1 ] = "on"

