import math
import pdb

def normalizeAngle(angle):
    while angle < -math.pi:
        angle += 2*math.pi
    while angle > math.pi:
        angle -= 2*math.pi
    return angle
    
def normalizeAngle0PI(angle):
    while angle < 0:
        angle += math.pi
    while angle > math.pi:
        angle -= math.pi
    return angle

    
def distAngles(a,b):
    
    lowerLimit = normalizeAngle(a)
    upperLimit = normalizeAngle(b)
    if lowerLimit > upperLimit:
        upperLimit += 2*math.pi
    
    diff = upperLimit - lowerLimit
    
    while diff < -2*math.pi:
        diff += 2*math.pi
    while diff > 2*math.pi:
        diff -= 2*math.pi
    
    
    #if diff > math.pi:
    #    diff -= math.pi
    #elif diff < -math.pi:
    #    diff += math.pi            
    
    return diff

def getMiddleAngle(a,b):
    
    lowerLimit = normalizeAngle(a)
    upperLimit = normalizeAngle(b)
    if lowerLimit > upperLimit:
        upperLimit += 2*math.pi
    
    result = (upperLimit + lowerLimit) / 2
    
    while result < -2*math.pi:
        result += 2*math.pi
    while result > 2*math.pi:
        result -= 2*math.pi
    
    return result




def distAnglesPIPI(a,b):
    
    lowerLimit = normalizeAngle(a)
    upperLimit = normalizeAngle(b)
    
    diff = upperLimit - lowerLimit
    if diff > math.pi:
        diff -= math.pi
    elif diff < -math.pi:
        diff += math.pi            
    
    return diff

def distAnglesUnorientedLine(a,b):
    
    lowerLimit = normalizeAngle0PI(a)
    upperLimit = normalizeAngle0PI(b)
    
    if upperLimit < lowerLimit:
        x = upperLimit
        upperLimit = lowerLimit
        lowerLimit = x
        
    diff = upperLimit - lowerLimit
    
    if diff > math.pi/2:
        diff = math.pi-diff
    
    return diff


def isAngleBetween(angle,lowerLimit,upperLimit):
    angle = normalizeAngle(angle)
    lowerLimit = normalizeAngle(lowerLimit)
    upperLimit = normalizeAngle(upperLimit)
    
    
    if lowerLimit > upperLimit:
        upperLimit += 2*math.pi
    
    if lowerLimit <= angle <= upperLimit:
        return True
    
    if lowerLimit <= angle + 2*math.pi <= upperLimit:
        return True
    
    return False
    
def distPose(pose1, pose2):
    return math.sqrt(math.pow(pose1.x - pose2.x,2) + math.pow(pose1.y - pose2.y,2))
	

if __name__ == '__main__':
    pdb.set_trace()
    print(distAngles(1.6,-2.04))        
