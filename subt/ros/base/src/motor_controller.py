from pid import PID
import pdb

#for anonymous objects
Object = lambda **kwargs: type("Object", (), kwargs)

class MotorController:
    def __init__(self, wheelBase,numberOfMotors):
        self.pidControllerFrontLeft = PID()
        self.pidControllerFrontRight = PID()
        self.pidControllerRearLeft = PID()
        self.pidControllerRearRight = PID()
        self.pidControllerCenterLeft = PID()
        self.pidControllerCenterRight = PID()
        self.wheelBase = wheelBase
        self.numberOfMotors = numberOfMotors
        self.lastForwardSpeed = 0

    def update(self,cmd_vel,actualWheelSpeed):
        #desiredSpeedFrontLeft = 0
        #desiredSpeedFrontRight = 0
        #desiredSpeedRearLeft = 0
        #desiredSpeedRearRight = 0
             

        desiredSpeed = Object    
        if self.numberOfMotors == 1:
            desiredSpeed.frontLeft = cmd_vel.linear.x
            desiredSpeed.frontRight = 0
            desiredSpeed.rearLeft = 0
            desiredSpeed.rearRight =0
            desiredSpeed.centerLeft = 0
            desiredSpeed.centerRight = 0
        else:
            desiredSpeed.frontLeft = cmd_vel.linear.x - cmd_vel.angular.z * self.wheelBase / 2
            desiredSpeed.frontRight = cmd_vel.linear.x + cmd_vel.angular.z * self.wheelBase / 2
            desiredSpeed.rearLeft = desiredSpeed.frontLeft 
            desiredSpeed.rearRight = desiredSpeed.frontRight
            desiredSpeed.centerLeft = desiredSpeed.frontLeft
            desiredSpeed.centerRight = desiredSpeed.frontRight
        
        newWheelSpeed = Object
        if desiredSpeed.frontLeft == 0 and\
           desiredSpeed.frontRight == 0 and\
           desiredSpeed.rearLeft == 0 and\
           desiredSpeed.rearRight == 0 and\
           desiredSpeed.centerLeft == 0 and\
           desiredSpeed.centerRight == 0:
            #robot wants to stop now
            newWheelSpeed.frontLeft = self.pidControllerFrontLeft.stop()
            newWheelSpeed.frontRight = self.pidControllerFrontRight.stop()
            newWheelSpeed.rearLeft = self.pidControllerRearLeft.stop()
            newWheelSpeed.rearRight = self.pidControllerRearRight.stop()
            newWheelSpeed.centerLeft = self.pidControllerCenterLeft.stop()
            newWheelSpeed.centerRight = self.pidControllerCenterRight.stop()
        elif (cmd_vel.linear.x > 0 and self.lastForwardSpeed < 0) or \
             (cmd_vel.linear.x < 0 and self.lastForwardSpeed > 0):
            #robot wants to change direction -> stop first. 
            newWheelSpeed.frontLeft = self.pidControllerFrontLeft.stop()
            newWheelSpeed.frontRight = self.pidControllerFrontRight.stop()
            newWheelSpeed.rearLeft = self.pidControllerRearLeft.stop()
            newWheelSpeed.rearRight = self.pidControllerRearRight.stop()
            newWheelSpeed.centerLeft = self.pidControllerCenterLeft.stop()
            newWheelSpeed.centerRight = self.pidControllerCenterRight.stop()
        else:                
            newWheelSpeed.frontLeft = self.pidControllerFrontLeft.update(desiredSpeed.frontLeft,actualWheelSpeed.frontLeft)
            newWheelSpeed.frontRight = self.pidControllerFrontRight.update(desiredSpeed.frontRight,actualWheelSpeed.frontRight)
            newWheelSpeed.rearLeft = self.pidControllerRearLeft.update(desiredSpeed.rearLeft,actualWheelSpeed.rearLeft)
            newWheelSpeed.rearRight = self.pidControllerRearRight.update(desiredSpeed.rearRight,actualWheelSpeed.rearRight)
            newWheelSpeed.centerLeft = self.pidControllerCenterLeft.update(desiredSpeed.centerLeft,actualWheelSpeed.centerLeft)
            newWheelSpeed.centerRight = self.pidControllerCenterRight.update(desiredSpeed.centerRight,actualWheelSpeed.centerRight)
            
            
            
            print "FL:\tdesired=%lf;\tactual=%lf;\tnew=%lf" % (desiredSpeed.frontLeft,actualWheelSpeed.frontLeft,newWheelSpeed.frontLeft)
            print "FR:\tdesired=%lf;\tactual=%lf;\tnew=%lf" % (desiredSpeed.frontRight,actualWheelSpeed.frontRight,newWheelSpeed.frontRight)
            print "RL:\tdesired=%lf;\tactual=%lf;\tnew=%lf" % (desiredSpeed.rearLeft,actualWheelSpeed.rearLeft,newWheelSpeed.rearLeft)
            print "RR:\tdesired=%lf;\tactual=%lf;\tnew=%lf" % (desiredSpeed.rearRight,actualWheelSpeed.rearRight,newWheelSpeed.rearRight)
            print "CL:\tdesired=%lf;\tactual=%lf;\tnew=%lf" % (desiredSpeed.centerLeft,actualWheelSpeed.centerLeft,newWheelSpeed.centerLeft)
            print "CR:\tdesired=%lf;\tactual=%lf;\tnew=%lf" % (desiredSpeed.centerRight,actualWheelSpeed.centerRight,newWheelSpeed.centerRight)
            
        self.lastForwardSpeed = cmd_vel.linear.x
        """
        newWheelSpeed.frontLeft = 0
        newWheelSpeed.frontRight = 0
        newWheelSpeed.rearLeft = 0
        newWheelSpeed.rearRight = 0
        newWheelSpeed.centerLeft = 0
        newWheelSpeed.centerRight = 0
        """
        return newWheelSpeed
        
        
        
