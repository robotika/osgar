import pdb
import math
import sys
from robohw_real_mob import RoboHWRealMob
from motor_controller import MotorController
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import struct

TICK_DELAY = 0.05 #0.1 #how many seconds lasts one synchronization cycle
TICK_DELAY = 2.0
TIMER_TICKS_PER_SECOND = 500
ANGULAR_FACTOR = 130
FORWARD_FACTOR = 100
COMPASS_VALUE_ZERO = 0#1600 degree
ANGULAR_CENTER = 120
NUMBER_OF_MOTORS = 4
WGS84Zone = None

#for anonymous objects
Object = lambda **kwargs: type("Object", (), kwargs)


class RoboconfigMob:
    def __init__(self):
        self.wheelBase = 0.24
        self.encDist1000MM = 6586
        self.lastEncoders = [0,0,0,0,0,0]
        self.hardware = RoboHWRealMob()
        self.motorController = MotorController(self.wheelBase,NUMBER_OF_MOTORS)
        self.lastTimer = 0
        self.lastMasterTime = 0
        #self.lastCompass = 0    
        cmd_vel = Twist()
        self.lastSpeeds = None
        self.wheelSpeeds = None
        self.update(cmd_vel)
        
    def update(self,cmd_vel):
        #send cmd to robot and return current status
        
        if self.hardware.getReboot():
            #hardware was rebooted
            self.hardware.setReboot(False)
            cmd_vel.linear.x = 0
            cmd_vel.angular.z = 0

        newWheelSpeed = self.motorController.update(cmd_vel,self.wheelSpeeds)
        speeds = self.convertSpeedToRobot(newWheelSpeed)
        executeAt = 0xFFFF & self.convertTimeToRobot(self.lastMasterTime + TICK_DELAY)
        timer,encoders,redSwitch = self.hardware.synchronize(executeAt,speeds)
        
        #############
        #status.setRedSwitch(hwStatus.getRedSwitch())  #how to represent red switch in ROS?
        actualTickDelay = self.convertTimeFromRobot(0xFFFF & (timer - self.lastTimer))
        speeds = self.convertSpeedFromRobot(encoders,self.lastEncoders,actualTickDelay)
        #newWheelSpeed = self.convertWheelSpeedFromRobot(hwStatus.getEncoders(),self.lastEncoders,actualTickDelay)
        #status.setWheelSpeed(newWheelSpeed)
        masterTime = self.lastMasterTime + actualTickDelay
        #status.setCompass(self.convertCompassFromRobot(hwStatus.getCompass()))
        ##############

        self.lastMasterTime = masterTime        
        self.lastTimer = timer
        self.lastEncoders = encoders
        self.lastSpeeds = speeds
        return redSwitch,speeds        

        
    def convertSpeedFromRobot(self,encoders,lastEncoders,time):
        encDiff = [0,0,0,0,0,0]
        
        #print "Encoders=",encoders
        for i in range(0,len(encoders)):
            encDiff[i] = encoders[i] - lastEncoders[i]
            unpacked = struct.unpack("h","%c%c" %(encDiff[i] % 256,0xFF& (encDiff[i] >> 8)))
            encDiff[i] = float(unpacked[0])
        #print "EncDiff=",encDiff, " Time=",time        
        encDiffL = (encDiff[1] + encDiff[0]) / 2
        encDiffR = -(encDiff[3] + encDiff[2]) / 2
        self.wheelSpeeds = Object
        self.wheelSpeeds.rearLeft = encDiff[1] / self.encDist1000MM / time
        self.wheelSpeeds.frontLeft = encDiff[0] / self.encDist1000MM / time
        self.wheelSpeeds.rearRight = -encDiff[3] / self.encDist1000MM / time
        self.wheelSpeeds.frontRight = -encDiff[2] / self.encDist1000MM / time

        speeds = Object
        speeds.fwd = ((encDiffL + encDiffR)/2.0)/self.encDist1000MM / time
        speeds.ang = (encDiffR - encDiffL)/(self.wheelBase * self.encDist1000MM) / time
        return speeds
        
    def convertSpeedToRobot(self,speed):        
        speeds = Object
        """
        speeds.frontRight = int(max(0, min(64 - speed.frontRight * 64,128)))
        speeds.frontLeft = int(max(128, min(192 - speed.frontLeft * 64,255)))
        speeds.rearLeft = int(max(0, min(64 - speed.rearRight * 64,128)))
        speeds.rearRight = int(max(128, min(192 - speed.rearLeft * 64,255)))
        speeds.centerLeft = int(max(0, min(64 - speed.centerRight * 64,128)))
        speeds.centerRight = int(max(128, min(192 - speed.centerLeft * 64,255)))
        """
        MIN = 78
        MAX = 178
        MID = 128
        DIFF = MID - MIN
        speeds.frontRight = int(max(MIN, min(MID - speed.frontRight * DIFF,MAX)))
        speeds.frontLeft = int(max(MIN, min(MID - speed.frontLeft * DIFF,MAX)))
        speeds.rearLeft = int(max(MIN, min(MID - speed.rearRight * DIFF,MAX)))
        speeds.rearRight = int(max(MIN, min(MID - speed.rearLeft * DIFF,MAX)))
        speeds.centerLeft = int(max(MIN, min(MID - speed.centerRight * DIFF,MAX)))
        speeds.centerRight = int(max(MIN, min(MID - speed.centerLeft * DIFF,MAX)))
        return speeds
   
    
    def convertTimeToRobot(self,seconds):
        return int(seconds * TIMER_TICKS_PER_SECOND)
                       
    def convertTimeFromRobot(self,timer):
        return float(timer) / TIMER_TICKS_PER_SECOND

