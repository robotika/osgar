import pdb
import math
import sys
import robomath
from robohw_real_car import RoboHWRealCar
from motor_controller import MotorController
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import struct

TICK_DELAY = 0.05 #0.1 #how many seconds lasts one synchronization cycle
TIMER_TICKS_PER_SECOND = 500
ANGULAR_FACTOR = 130
FORWARD_FACTOR = 100
COMPASS_VALUE_ZERO = 1520#1600 degree
ANGULAR_CENTER = 120
NUMBER_OF_MOTORS = 1
WGS84Zone = None


#for anonymous objects
Object = lambda **kwargs: type("Object", (), kwargs)


class RoboconfigCar:
    def __init__(self):
        self.wheelBase = 0.235
        self.encDist1000MM = 13133
        self.lastEncoders = [0,0,0,0]
        self.hardware = RoboHWRealCar()
        self.motorController = MotorController(self.wheelBase,NUMBER_OF_MOTORS)
        self.lastTimer = 0
        self.lastMasterTime = 0
        #self.lastCompass = 0    
        cmd_vel = Twist()
        self.lastSpeeds = None
        self.wheelSpeeds = None
        self.lastCompass = 0
        self.redSwitch = False
        self.update(cmd_vel)
    
    def update(self,cmd_vel):
        #send cmd to robot and return current status
        
        if self.hardware.getReboot():
            #hardware was rebooted
            self.hardware.setReboot(False)
            cmd_vel.linear.x = 0
            cmd_vel.angular.z = 0
        if self.redSwitch:
            cmd_vel.linear.x = 0
            cmd_vel.angular.z = 0

        newWheelSpeed = self.motorController.update(cmd_vel,self.wheelSpeeds)
        speeds = self.convertSpeedToRobot(newWheelSpeed)
        direction = self.convertDirectionToRobot(cmd_vel)
        executeAt = 0xFFFF & self.convertTimeToRobot(self.lastMasterTime + TICK_DELAY)
        
        timer,encoders,self.redSwitch,compass = self.hardware.synchronize(executeAt,speeds.frontLeft,direction)
        
        #############
        #status.setRedSwitch(hwStatus.getRedSwitch())  #how to represent red switch in ROS?
        actualTickDelay = self.convertTimeFromRobot(0xFFFF & (timer - self.lastTimer))
        speeds = self.convertSpeedFromRobot(encoders,self.lastEncoders,actualTickDelay)
        speeds.ang = self.convertAngularSpeedFromRobot(self.convertCompassFromRobot(compass),actualTickDelay)
        #newWheelSpeed = self.convertWheelSpeedFromRobot(hwStatus.getEncoders(),self.lastEncoders,actualTickDelay)
        #status.setWheelSpeed(newWheelSpeed)
        masterTime = self.lastMasterTime + actualTickDelay
        heading = self.convertCompassFromRobot(compass)
        #print("Compass:",compass)
        ##############

        self.lastMasterTime = masterTime        
        self.lastTimer = timer
        self.lastEncoders = encoders
        self.lastSpeeds = speeds
        return self.redSwitch,speeds,heading        

    def convertAngularSpeedFromRobot(self,compass,time):
        compassDiff = robomath.normalizeAngle(compass - self.lastCompass) 
        result = compassDiff /time
        
        self.lastTime = time
        self.lastCompass = compass
        return result
   
    def convertCompassFromRobot(self,compass):
        if compass == -1:
            raise Exception("Compass not working")
        angleInRad = - math.pi / 180 * float(compass - COMPASS_VALUE_ZERO)/10
        normalized = robomath.normalizeAngle(angleInRad)
        return normalized
                                                                
    def convertSpeedFromRobot(self,encoders,lastEncoders,time):
        encDiff = [0,0,0,0]
        
        #print "Encoders=",encoders
        for i in range(0,len(encoders)):
            encDiff[i] = encoders[i] - lastEncoders[i]
            unpacked = struct.unpack("h","%c%c" %(encDiff[i] % 256,0xFF& (encDiff[i] >> 8)))
            encDiff[i] = float(unpacked[0])
        #print "EncDiff=",encDiff, " Time=",time        
        
        self.wheelSpeeds = Object
        self.wheelSpeeds.frontRight = 0
        self.wheelSpeeds.rearRight = 0
        self.wheelSpeeds.frontLeft = (-encDiff[0])/self.encDist1000MM / time
        self.wheelSpeeds.rearLeft = 0

        speeds = Object
        speeds.fwd = (-encDiff[0])/self.encDist1000MM / time
        speeds.ang = 0
        return speeds
        
    def convertSpeedToRobot(self,speed):        
        speeds = Object
        speeds.frontLeft = 128 + speed.frontLeft * FORWARD_FACTOR
        if speeds.frontLeft > 170:
            speeds.frontLeft = 170
        if speeds.frontLeft < 90:
            speeds.frontLeft = 90
        return speeds
   
    
    def convertTimeToRobot(self,seconds):
        return int(seconds * TIMER_TICKS_PER_SECOND)
                       
    def convertTimeFromRobot(self,timer):
        return float(timer) / TIMER_TICKS_PER_SECOND

    def convertDirectionToRobot(self,cmd):        
        if cmd.linear.x >= 0:
            direction = cmd.angular.z
        else:
            direction = -cmd.angular.z
        newDirection = ANGULAR_CENTER - direction * ANGULAR_FACTOR
        if newDirection > 205:
            newDirection = 205
        if newDirection < 50:
            newDirection = 50 
        return newDirection
    
