from __future__ import print_function
import math
import pdb
import time
import serial
import os
import rospy
import struct

PACKET_START = 0xAB
PACKET_DEBUG_START = 0xAD
PACKET_RETRY = 0xEE
ECHO_CHAR = ord('D')
AVR_REBOOT_CHAR = ord('R')
AVR_REBOOT_CHAR_MIN_COUNT = 3
SABERTOOTH_INIT_CHAR = 0xAA
SABERTOOTH_1_ADDRESS = 0x80
SABERTOOTH_2_ADDRESS = 0x81
SABERTOOTH_COMMAND_MOTOR_1 = 0x06
SABERTOOTH_COMMAND_MOTOR_2 = 0x07
MIN_SPEED_TO_SEND = -1
MAX_TIME_TO_SLEEP = -1
MAX_SPEED_TO_SEND = 255

#for anonymous objects
Object = lambda **kwargs: type("Object", (), kwargs)

class RoboHWRealMob:
    def __init__(self):
        self.connectArduino()
        #self.connectEstop()        
        
        self.encoders = [0,0,0,0]
        self.lastEncoders = [0,0,0,0]
        self.isFirstRun = True    
        self.lastSentWheelSpeed = Object
        self.lastSentWheelSpeed.frontLeftSpeed = 0
        self.lastSentWheelSpeed.frontRightSpeed = 0
        self.lastSentWheelSpeed.rearLeftSpeed = 0
        self.lastSentWheelSpeed.rearRightSpeed = 0
        self.lastSentIter = Object
        self.lastSentIter.frontRight = -999
        self.lastSentIter.frontLeft = -999
        self.lastSentIter.rearRight = -999
        self.lastSentIter.rearLeft = -999
        self.currentIter = 0
        self.reboot = False
        self.servo = 0
        self.robot = "Mobos2"

    def connectArduino(self):
        port = rospy.get_param('/base/port')
        try:
            #arduino micro is reset by opening port at 1200baud
            ser = serial.Serial(
                port,
                baudrate=1200,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS
            )
            ser.isOpen()
            ser.close()
            time.sleep(10.0)
            print("Initialize communication with Arduino")
            self.port = serial.Serial(port,baudrate=38400,timeout=0.05, parity=serial.PARITY_NONE,
                        stopbits=serial.STOPBITS_ONE,
                        bytesize=serial.EIGHTBITS)
        except:
            print("_____________________________")
            print("Robot port ",port," can't be found!")
            print("_____________________________")
            raise

    def connectEstop(self):
        port = "/dev/estop"
        try:
            self.estopPort = serial.Serial(port,baudrate=9600,timeout=0.05)
        except:
            print("_____________________________")
            print("Estop port ",port," can't be found!")
            print("_____________________________")
            raise

    def synchronize(self,executeAt, speeds,watchdog = 255):
        servos = [0,0,0,0]
        digitalOutputs = 0
        #pdb.set_trace()
        if self.isFirstRun:
            self.isFirstRun = False
            #wait for AVR reboot bytes
            print("Waiting for AVR:")
            while True:
                print("y.", end=' ')
                byte = self.waitForByte()
                if byte == chr(AVR_REBOOT_CHAR):
                    break
                    
            byte = self.waitForByte()
            while byte == chr(AVR_REBOOT_CHAR):
                byte = self.waitForByte()
            
            
        while True:
            self.sendByte(PACKET_START)
            executeAtByte0 = executeAt & 0xFF 
            executeAtByte1 = (executeAt >> 8) & 0xFF
            
            self.sendByte(executeAtByte0)
            self.sendByte(executeAtByte1)

            self.sendByte(speeds.frontLeft)
            self.sendByte(speeds.frontRight)
            self.sendByte(speeds.rearLeft)
            self.sendByte(speeds.rearRight)
            self.sendByte(speeds.centerRight)
            self.sendByte(speeds.centerLeft)
            self.sendByte(watchdog)
            self.sendByte(digitalOutputs)
            self.sendByte((executeAtByte0 + executeAtByte1 + speeds.rearLeft + speeds.rearRight + speeds.frontLeft + speeds.frontRight + speeds.centerLeft + speeds.centerRight + watchdog + digitalOutputs) & 0xFF)

            tmp = self.waitForByte()
            while tmp == chr(PACKET_DEBUG_START):
                #debug message has fixed length 4 bytes
                
                for i in range(0,4):
                    tmp = self.waitForByte()
                tmp = self.waitForByte()
                    
            if tmp != chr(PACKET_START):
                self.restoreCommunication()
            else:
                break
  
        bytes = []
        for i in range(0,2):
            bytes.append(self.waitForByte())
        timer = ord(bytes[0])
        timer += 256 * ord(bytes[1])
        
        bytes = []
        for i in range(0,12):
            bytes.append(self.waitForByte())
        
        R_encoder0 = ord(bytes[0])
        R_encoder0 += 256 * ord(bytes[1])
        R_encoder1 = ord(bytes[2])
        R_encoder1 += 256 * ord(bytes[3])
        R_encoder2 = ord(bytes[4])
        R_encoder2 += 256 * ord(bytes[5])
        R_encoder3 = ord(bytes[6])
        R_encoder3 += 256 * ord(bytes[7])
        R_encoder4 = ord(bytes[8])
        R_encoder4 += 256 * ord(bytes[9])
        R_encoder5 = ord(bytes[10])
        R_encoder5 += 256 * ord(bytes[11])

            
        #print "Bytes:",bytes
        
        R_digitalInputs = ord(self.waitForByte())
        redSwitch =  R_digitalInputs == 3
    
            
        rangefinders = []
        for i in range(0,4):
            rangefinders.append(ord(self.waitForByte()))
            
        
        R_compass = ord(self.waitForByte())
        R_compass += 255 * ord(self.waitForByte())
        checkSum = ord(self.waitForByte())
        
        """
        #------------------------------
        #send command to sabertooth controller 1
        #motor1
        speed = int(speeds.frontLeft)
        #print("Speed FL=%lf"%(speed))
        special_characters = (PACKET_START,ECHO_CHAR)
        if abs(speed - self.lastSentWheelSpeed.frontLeft) > MIN_SPEED_TO_SEND or \
            self.currentIter - self.lastSentIter.frontLeft > MAX_TIME_TO_SLEEP:
            if speed > MAX_SPEED_TO_SEND:
                speed = MAX_SPEED_TO_SEND 
            checksum = (SABERTOOTH_1_ADDRESS + SABERTOOTH_COMMAND_MOTOR_1 + speed) & 0x7F
            while checksum in special_characters or speed in special_characters:
                speed += 1
                checksum = (SABERTOOTH_1_ADDRESS + SABERTOOTH_COMMAND_MOTOR_1 + speed) & 0x7F
            self.sendByte(int(SABERTOOTH_1_ADDRESS))
            self.sendByte(int(SABERTOOTH_COMMAND_MOTOR_1))
            self.sendByte(int(speed))
            self.sendByte(int(checksum))
            self.lastSentWheelSpeed.frontLeft = speed
            self.lastSentIter.frontLeft = self.currentIter
        
        #motor2
        speed = int(speeds.rearLeft)
        #print("Speed RL=%lf"%(speed))
         
        if abs(speed - self.lastSentWheelSpeed.rearLeft) > MIN_SPEED_TO_SEND or \
            self.currentIter - self.lastSentIter.rearLeft > MAX_TIME_TO_SLEEP:
            if speed > MAX_SPEED_TO_SEND:
                speed = MAX_SPEED_TO_SEND 
            checksum = (SABERTOOTH_1_ADDRESS + SABERTOOTH_COMMAND_MOTOR_2 + speed) & 0x7F
            while checksum in special_characters or speed in special_characters:
                speed += 1
                checksum = (SABERTOOTH_1_ADDRESS + SABERTOOTH_COMMAND_MOTOR_2 + speed) & 0x7F
            
            self.sendByte(int(SABERTOOTH_1_ADDRESS))
            self.sendByte(int(SABERTOOTH_COMMAND_MOTOR_2))
            self.sendByte(int(speed))
            self.sendByte(int(checksum))
            self.lastSentWheelSpeed.rearLeft = speed
            self.lastSentIter.rearLeft = self.currentIter
        
        #------------------------------
        #send command to sabertooth controller 2
        #motor1
        speed = int(speeds.frontRight)
        #print("Speed FR=%lf"%(speed))
        
        if abs(speed - self.lastSentWheelSpeed.frontRight) > MIN_SPEED_TO_SEND or \
            self.currentIter - self.lastSentIter.frontRight > MAX_TIME_TO_SLEEP:
            if speed > MAX_SPEED_TO_SEND:
                speed = MAX_SPEED_TO_SEND 
            checksum = (SABERTOOTH_2_ADDRESS + SABERTOOTH_COMMAND_MOTOR_1 + speed) & 0x7F
            try:
                while checksum in special_characters or speed in special_characters:
                    speed += 1
                    checksum = (SABERTOOTH_2_ADDRESS + SABERTOOTH_COMMAND_MOTOR_1 + speed) & 0x7F
            except:
                pdb.set_trace()
                
            self.sendByte(int(SABERTOOTH_2_ADDRESS))
            self.sendByte(int(SABERTOOTH_COMMAND_MOTOR_1))
            self.sendByte(int(speed))
            self.sendByte(int(checksum))
            self.lastSentWheelSpeed.frontRight = speed
            self.lastSentIter.frontRight = self.currentIter
        
        #motor2
        speed = int(speeds.rearRight)
        #print("Speed RR=%lf"%(speed))
        
        if abs(speed - self.lastSentWheelSpeed.rearRight) > MIN_SPEED_TO_SEND or \
            self.currentIter - self.lastSentIter.rearRight > MAX_TIME_TO_SLEEP:
            if speed > MAX_SPEED_TO_SEND:
                speed = MAX_SPEED_TO_SEND 
            checksum = (SABERTOOTH_2_ADDRESS + SABERTOOTH_COMMAND_MOTOR_2 + speed) & 0x7F
            while checksum in special_characters or speed in special_characters:
                speed += 1
                checksum = (SABERTOOTH_2_ADDRESS + SABERTOOTH_COMMAND_MOTOR_2 + speed) & 0x7F
            
            self.sendByte(int(SABERTOOTH_2_ADDRESS))
            self.sendByte(int(SABERTOOTH_COMMAND_MOTOR_2))
            self.sendByte(int(speed))
            self.sendByte(int(checksum))
            self.lastSentWheelSpeed.rearRight = speed
            self.lastSentIter.rearRight = self.currentIter
        
        #-------------------------------
        """
        return timer,[R_encoder0,R_encoder1,R_encoder2,R_encoder3,R_encoder4,R_encoder5],redSwitch




    def restoreCommunication(self):
        while True:
            self.sendByte(ECHO_CHAR)
            tmp = self.waitForByte()
            if tmp == chr(ECHO_CHAR):
                break
            elif tmp == chr(AVR_REBOOT_CHAR):
                count = 1
                tmp = self.waitForByte()
                while tmp == chr(AVR_REBOOT_CHAR):
                    count += 1
                    tmp = self.waitForByte()
                if count > AVR_REBOOT_CHAR_MIN_COUNT:
                    #reboot of AVR detected
                    self.setReboot(True)
                    #send sabertooth initialization byte for setting up the baudrate
                    time.sleep(2.0)
                    self.sendByte(SABERTOOTH_INIT_CHAR)
                    
                    
                
            print("x.", end=' ')
            
        while self.waitForByte() != -1:
            pass
        
        
    def sendByte(self,byte):
        byte = struct.pack("B",byte)
        #rospy.logdebug("Sending:%02x(%s) " % (ord(byte),byte))
        self.port.write(byte)
        #rospy.logdebug(">%02x(%s) " % (ord(byte),byte))
        #print(">%02x(%s) " % (ord(byte),byte))
        
    def waitForByte(self):
        byte = self.port.read()
        if byte == b"":
            rospy.logdebug("Comm timeout")
            return -1
        #print("<%02x(%s) " % (ord(byte),byte))
        #rospy.logdebug("<%02x(%s) " % (ord(byte),byte))
        return byte

    def getReboot(self):
        return self.reboot
        
    def setReboot(self,value):
        self.reboot = value
    

