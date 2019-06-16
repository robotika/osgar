"""
  Extra math routines
"""
import math


def normalizeAnglePIPI( angle ):
    while angle < -math.pi:
        angle += 2*math.pi
    while angle > math.pi:
        angle -= 2*math.pi
    return angle 

def distAngles(a,b):
    lowerLimit = normalizeAnglePIPI(a)
    upperLimit = normalizeAnglePIPI(b)
    if lowerLimit > upperLimit:
        upperLimit += 2*math.pi
    
    diff = upperLimit - lowerLimit
    while diff < -2*math.pi:
        diff += 2*math.pi
    while diff > 2*math.pi:
        diff -= 2*math.pi
    return diff

def isAngleBetween(angle,lowerLimit,upperLimit):
    angle = normalizeAnglePIPI(angle)
    lowerLimit = normalizeAnglePIPI(lowerLimit)
    upperLimit = normalizeAnglePIPI(upperLimit)
    if lowerLimit > upperLimit:
        upperLimit += 2*math.pi
    
    if lowerLimit <= angle <= upperLimit:
        return True
    
    if lowerLimit <= angle + 2*math.pi <= upperLimit:
        return True
    
    return False

def getMiddleAngle(a,b):
    
    lowerLimit = normalizeAnglePIPI(a)
    upperLimit = normalizeAnglePIPI(b)
    if lowerLimit > upperLimit:
        upperLimit += 2*math.pi
    
    result = (upperLimit + lowerLimit) / 2
    
    while result < -2*math.pi:
        result += 2*math.pi
    while result > 2*math.pi:
        result -= 2*math.pi
    
    return result


# vim: expandtab sw=4 ts=4
