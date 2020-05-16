"""
  Moon Skyline analysis
"""
import math

import numpy as np


def skyline(img):
    b = img[:,:,0]
    g = img[:,:,1]
    r = img[:,:,2]
    mask = r > 10
    return mask.argmax(axis=0)


def draw_skyline(img, skyline):
    # TODO optimize - there is surely many times faster version, but this is just for debug
    img2 = img.copy()
    for x, y in enumerate(skyline):
        img2[y, x, 0] = 0
        img2[y, x, 1] = 255
        img2[y, x, 2] = 0
    return img2


# vim: expandtab sw=4 ts=4
