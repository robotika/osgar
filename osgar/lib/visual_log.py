import pdb
import cv2
import numpy as np
import math
import shutil
import time
import os

SCALE = 1.0
FRAME_WIDTH = 400
FRAME_HEIGHT = 400
CAR_SCALE = 0.15
PIXELS_PER_METER = 40
BORDER = 10
HEADING_SHIFT = -math.pi/2

class VisualLog:
    def __init__(self):
        self.carImage = cv2.resize(cv2.imread("car_clipart.png"),(0,0),fx=CAR_SCALE,fy=CAR_SCALE)
        self.carHeight, self.carWidth = self.carImage.shape[:2]
        self.carCenter = (self.carWidth / 2, self.carHeight / 2)
        self.image = None
        self.frameIndex = 0
        self.border = BORDER
            
    def getPixelsPerMeter(self):
        return PIXELS_PER_METER
    
    def getFrameWidth(self):
        return FRAME_WIDTH
    
    def getFrameHeight(self):
        return FRAME_HEIGHT
        
    def getImage(self):
        return self.image
        

    def init(self):
        #self.image = np.full((FRAME_HEIGHT,FRAME_WIDTH,3),255,dtype = np.uint8)
        self.image = np.zeros((FRAME_HEIGHT,FRAME_WIDTH,3),dtype = np.uint8)
        
    def drawCar(self,pose):
        radians = pose[2] + HEADING_SHIFT
        angle = math.degrees(radians)
        sin = math.sin(radians)
        cos = math.cos(radians)
        boundWidth = int((self.carHeight * abs(sin)) + (self.carWidth * abs(cos)))
        boundHeight = int((self.carHeight * abs(cos)) + (self.carWidth * abs(sin)))
        rotationMat = cv2.getRotationMatrix2D(self.carCenter, angle, 1)
        rotationMat[0, 2] += ((boundWidth / 2) - self.carCenter[0])
        rotationMat[1, 2] += ((boundHeight / 2) - self.carCenter[1])
        carRotatedImage = cv2.warpAffine(self.carImage, rotationMat, (boundWidth, boundHeight),borderMode = cv2.BORDER_CONSTANT, borderValue = (255,255,255))
        foreground = np.full(self.image.shape,[255,255,255],dtype = np.uint8)
        xOffset = (FRAME_WIDTH - boundWidth)/2 + self.border
        yOffset = (FRAME_HEIGHT - boundHeight)/2 + self.border
        
        foreground[int(yOffset):int(yOffset + boundHeight), int(xOffset):int(xOffset+boundWidth)] = carRotatedImage
        maskChannel = np.array((foreground[:,:,0] < 150 ),dtype = np.uint8)
        mask = np.zeros(foreground.shape,dtype = np.uint8)
        mask[:,:,0] = maskChannel
        mask[:,:,1] = maskChannel
        mask[:,:,2] = maskChannel
        
        # Multiply the foreground with the alpha matte
        foreground = cv2.multiply(mask, foreground)
        # Multiply the background with ( 1 - alpha )
        background = cv2.multiply(1 - mask, self.image)
        # Add the masked foreground and background.
        self.image = cv2.add(foreground, background)

    
    def show(self):
        cv2.imshow('VisualLog', self.image)
        cv2.waitKey(1)
        pass
