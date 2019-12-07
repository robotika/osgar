import pdb


PID_P = 0.3 # MOB # was 0.1
PID_D = 0.0 # MOB
PID_I = 0
MAX_FORCE = 0.4 

class PID:
    def __init__(self):
        self.prevDifference = 0
        self.prevSpeed = 0
        self.MAX_FORCE = MAX_FORCE
        self.PID_P = PID_P
        self.PID_D = PID_D
        self.PID_I = PID_I        

        
    def update(self,desiredWheelSpeed,actualWheelSpeed):
        
        actualDifference = desiredWheelSpeed - actualWheelSpeed
        
        newSpeedIncrement = actualDifference * self.PID_P - (self.prevDifference - actualDifference) * self.PID_D + self.PID_I

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
        return 0
