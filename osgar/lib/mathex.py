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

# vim: expandtab sw=4 ts=4
