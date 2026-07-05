import math
import pdb
import serial
import time
import os

 

class Compass():
    def __init__(self):
        self.port = serial.Serial("/dev/ttyUSB0",baudrate=19200,timeout=5)
        self.lastCompass = -1
        
    def update(self):

        self.port.write(b"\x55\xC1\x02\x02")
        data1 = self.port.read()
        data2 = self.port.read()
        value = ord(data1) * 256 + ord(data2)
        self.lastCompass = value
        return value
