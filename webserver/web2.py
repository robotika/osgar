#!/usr/bin/env python
"""
  webserver with connection to serial controller
  Usage:
     ./web2.py [<COM port>|dummy]
"""

import string,cgi,time
import os
import sys
from BaseHTTPServer import BaseHTTPRequestHandler, HTTPServer
from mimetypes import guess_type

from controller import Controller, DummyController

rootdir = os.path.dirname(os.path.abspath(__file__))

class MyHandler(BaseHTTPRequestHandler):
  def do_GET(self):
    cmd = None
    print "MD:", self.path
    s = self.path.split('/')
    if len(s) > 0 and s[0] == '':
      s = s[1:]
    if len(s) >= 3 and s[0]=='api' and s[1]=='0.1':
      print s
      nodeId = int(s[2])
      if nodeId < len(self.server.light):
        if len(s) > 3:
          cmd = s[3]
          if cmd in ['on','off']:
            self.server.light[ nodeId ] = cmd
            if self.server.controller:
              self.server.controller.send( nodeId, cmd )
            self.send_response(200)
            self.send_header("Content-Type", "text/plain")
            self.end_headers()
            self.wfile.write( "OK" )
            return
        else:
          self.send_response(200)
          self.send_header("Content-Type", "text/plain")
          self.end_headers()
          if self.server.controller:
            self.server.light[ nodeId ] = self.server.controller.query( nodeId )
          self.wfile.write( self.server.light[ nodeId ] )
          return

      self.send_response(200)
      self.send_header("Content-Type", "text/plain")
      self.end_headers()
      self.wfile.write( "ERROR" )
      return
    if self.path == '/' or cmd != None:
      self.send_response(200)
      self.send_header("Content-Type", "text/html")
      self.end_headers()
      self.wfile.write( open("lights.html").read() )
      return

    ctype, encoding = guess_type(self.path)
    print ctype, encoding
    self.send_response(200)
    if ctype:
      self.send_header("Content-Type", ctype)
    if encoding:
      self.send_header("Content-Encoding", encoding) 
    
    if self.path.endswith( ".png" ):
      ctype, encoding = guess_type(self.path)
    self.end_headers()
    self.wfile.write( open(rootdir+os.sep+self.path, "rb").read() )

def main( comName ):
  try:
    server = HTTPServer( ('',8888), MyHandler )
    server.light = 5*['off'] # maybe threestate, unknown, on, off OR even request-for-on, request-for-off
    server.controler = None
    if comName == "dummy":
      server.controller = DummyController()
    else:
      server.controller = Controller( comName )
    print 'started httpserver...'
    server.serve_forever()
  except KeyboardInterrupt:
    print 'keyboard interrupt'
    server.socket.close()

if __name__ == '__main__':
  if len(sys.argv) < 2:
    print __doc__
    sys.exit(1)
  main( sys.argv[1] )

