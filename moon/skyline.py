"""
  Moon Skyline analysis
"""
import math

import numpy as np


def skyline(img):
    b = img[:,:,0]
    g = img[:,:,1]
    r = img[:,:,2]
    mask = r > 50
    return mask.argmax(axis=0)


# vim: expandtab sw=4 ts=4
