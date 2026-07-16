import pdb


PID_P = 0.2 # MOB # was 0.1
PID_D = 0.05 # MOB
PID_I = 0.05
MAX_FORCE = 0.6 
MAX_INTEGRATOR = 0.3

class PID:
    def __init__(self):
        self.prevDifference = 0
        self.prevSpeed = 0
        self.MAX_FORCE = MAX_FORCE
        self.PID_P = PID_P
        self.PID_D = PID_D
        self.PID_I = PID_I        
        self.integrator = 0
        
    def update(self,desiredWheelSpeed,actualWheelSpeed):
        
        actualDifference = desiredWheelSpeed - actualWheelSpeed
        error = self.prevDifference - actualDifference
        self.integrator += error

        if self.integrator * PID_I > MAX_INTEGRATOR:
            self.integrator = MAX_INTEGRATOR / PID_I
        elif self.integrator < -MAX_INTEGRATOR / PID_I:
            self.integrator = -MAX_INTEGRATOR / PID_I

        newSpeedIncrement = actualDifference * self.PID_P - (self.prevDifference - actualDifference) * self.PID_D + self.PID_I * self.integrator

        self.prevDifference = actualDifference
        
        newSpeed = self.prevSpeed + newSpeedIncrement
        
        self.prevSpeed = newSpeed
        
        if newSpeed > self.MAX_FORCE:
            newSpeed = self.MAX_FORCE
        elif newSpeed < -self.MAX_FORCE:
            newSpeed = -self.MAX_FORCE  

        
        self.prevSpeed = newSpeed
        return newSpeed

    def stop(self):
        self.prevDifference = 0
        self.prevSpeed = 0
        self.integrator = 0
        return 0
