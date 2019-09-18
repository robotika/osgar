import math
import pdb
import sys
import os
import time
import cv2
import numpy as np
import numpy.linalg as linalg
import pygame

class CameraThresholding():
    def __init__(self):
        self.frameWidth = 320
        self.frameHeight = 240
        
    
    def update(self,image):
        #  create a copy of the surface
        view = pygame.surfarray.array3d(image)

        #  convert from (width, height, channel) to (height, width, channel)
        view = view.transpose([1, 0, 2])

        #  convert from rgb to bgr
        image = cv2.cvtColor(view, cv2.COLOR_RGB2BGR)
        
        originalImage = cv2.medianBlur(image,5) 
        
        #grass
        b = originalImage[:,:,0]
        g = originalImage[:,:,1]
        r = originalImage[:,:,2]
        
        tmp1 = r < 60
        tmp2 = g < 60
        tmp3 = b < 50
        road = np.logical_and(tmp1,np.logical_and(tmp2,tmp3))
        
        road = np.array(road*255,dtype = np.uint8)
        cv2.imshow("Road", road)
        # find moments of the image
        M = cv2.moments(road)
        centerX = int(M['m10']/M['m00'])
        centerY = int(M['m01']/M['m00'])
        direction = math.atan2(centerY,centerX) - math.pi/2
        print("centerX=%lf \t centerY=%lf \t Direction=%lf"%(centerX,centerY,direction))
        cv2.circle(image,(centerX,centerY),5,(0,0,255))
        cv2.line(image, (int(centerX),int(centerY)),(int(self.frameWidth/2),int(self.frameHeight)),(0,0,255),3)

        cv2.imshow("OriginalImage", image)
        
        birdsImage = np.zeros(shape=(self.frameWidth, self.frameHeight), dtype="uint16")
        birdsImage.fill(128)
        cv2.waitKey(1)
        
        return
    
    