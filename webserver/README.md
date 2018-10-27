Python Webserver for IQ80 Network - version 0

Installation (win32)

1) Install Python
for example http://python.org/ftp/python/2.7.2/python-2.7.2.msi

2) Install PySerial
http://sourceforge.net/projects/pyserial/files/pyserial/2.5/pyserial-2.5.win32.exe/download?source=files

3) Download jquery.js 
http://ajax.googleapis.com/ajax/libs/jquery/1.6.4/jquery.min.js
(this is for installation without internet connection - otherwise you can modify lights.html for direct download)

4) Test dummy webserver:
  files: 
     web2.py - web server
     controller.py - connection to serial line
     lights.html - main webpage
     jquery.js - java script for main page

  Run:  web2.py dummy
- it starts webserver without connection to serial port
- in Mozilla set URL to http://localhost/ and you should see four rooms

5) Test webserver with COM:
  Run:  web2.py COM6 (for example if COM6 is the connected COM port)
  On 9600 it sends readable ASCII commands
  For status info is used simple "procotol" 'a'..'z' lights off, 'A'..'Z' lights on

