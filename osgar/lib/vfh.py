import numpy as np
import pdb
import math
import osgar.lib.mathex as mathex
import cv2

HEADING_SHIFT = math.pi/2
DIST_THRESHOLD = 5   #m - radius of local map that VFH considers
DISTANCE_MATRIX_ZERO_THRESHOLD = 10 #was 20
DISTANCE_MATRIX_MAX_DIST_THRESHOLD = 140 #px
WIDE_OPENING_GRAVITY = math.pi / 5 
OPENING_THRESHOLD = 0.1 + 2*WIDE_OPENING_GRAVITY
DIRECTION_OF_WAYPOINT_FACTOR = 1
CURRENT_DIRECTION_FACTOR = 0.2
LAST_DIRECTION_FACTOR = 0.2
HEADING_SHIFT_DEGREES = math.degrees(HEADING_SHIFT)

        
class VFH:
    def __init__(self,horizontalLidarOcgm,visualLog):
        self.ocgmCellsPerMeter = horizontalLidarOcgm.ocgmCellsPerMeter
        self.visualLog = visualLog
        #initialize proportional distance matrix
        globalObstacleMap = horizontalLidarOcgm._map
        subFrame = self.getSubFrameSingleChannel(globalObstacleMap, DIST_THRESHOLD)
        self.distanceMatrix = np.zeros(subFrame.shape,dtype=np.float32)            
        centerX = int(self.distanceMatrix.shape[0]/2)
        centerY = int(self.distanceMatrix.shape[1]/2)
        self.lastDirection = 0
        
        for x in range(self.distanceMatrix.shape[0]):
            for y in range(self.distanceMatrix.shape[1]):
                dist =  int(math.sqrt(math.pow(x - centerX,2) + math.pow(y - centerY,2)))
                if dist < DISTANCE_MATRIX_ZERO_THRESHOLD:
                    dist = centerX
                if dist > DISTANCE_MATRIX_MAX_DIST_THRESHOLD:
                    dist = DISTANCE_MATRIX_MAX_DIST_THRESHOLD
                self.distanceMatrix[x][y] = dist    
        self.distanceMatrix /= DISTANCE_MATRIX_MAX_DIST_THRESHOLD #normalize distance
        self.distanceMatrix = 1 - self.distanceMatrix
        
        
        
    def update(self,globalWaypoint, pose, localMap):
        directionOfWaypoint = mathex.normalizeAnglePIPI(math.atan2(globalWaypoint[1] - pose[1],
                               globalWaypoint[0] - pose[0])  - HEADING_SHIFT)
            
        #polar histogram sectors have directions relative to the robot frame
        histogram = self.createPrimaryPolarHistogram(localMap,pose)                            
        histogram = self.createBinaryPolarHistogram(histogram)
        openings = self.findOpenings(histogram)            
        candidateDirections,isWide = self.findCandidateDirections(openings,directionOfWaypoint)
        desiredDirection = self.getBestDirection(candidateDirections,directionOfWaypoint)
        self.drawVFHToVisualLog(histogram,candidateDirections,desiredDirection)
                                
    def createPrimaryPolarHistogram(self,obstacleMap,currentPose):
        subFrame = self.getSubFrameSingleChannel(obstacleMap,DIST_THRESHOLD)
        #cv2.imshow('getSubFrameSingleChannel', subFrame) #for debug
        element = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(10,10))
        subFrame = cv2.dilate(subFrame,element)
        subFrame = cv2.medianBlur(subFrame,5) 
        #cv2.imshow('subFrame', subFrame) #for debug
        #cv2.imshow('distanceMatrix', self.distanceMatrix) #for debug
        subFrame = cv2.multiply(subFrame,self.distanceMatrix)
        #cv2.imshow('subFrame multiplied', subFrame) #for debug
        binary = subFrame > 0.3
        #cv2.imshow('VFH binary', np.array(binary * 255,dtype = np.uint8)) #for debug
        polar = cv2.linearPolar(np.array(binary * 255,dtype = np.uint8), (binary.shape[1]/2,binary.shape[0]/2),binary.shape[0]/2, cv2.WARP_FILL_OUTLIERS)
        #cv2.imshow('VFH polar', polar) #for debug
        flippedPolar = np.flipud(polar)
        #cv2.imshow('Flipped polar', flippedPolar) #for debug
        rolledPolar = np.roll(flippedPolar,-int(polar.shape[0]/4),0)
        #cv2.imshow('rolledPolar', rolledPolar) #for debug
        blockedSegments = np.sum(rolledPolar,axis=1)
        return blockedSegments
        
    def getSubFrameSingleChannel(self,localMap, radius):
        #convert radius from meters to gridmap pixels
        radius *= self.ocgmCellsPerMeter       
        mapWidth,mapHeight = localMap.shape
        subframe = localMap[int(mapHeight/2-radius):int(mapHeight/2+radius),int(mapWidth/2-radius):int(mapWidth/2+radius)]
        
        return subframe
    
    def createBinaryPolarHistogram(self,histogram):
        #create binary polar histogram
        return histogram > 0
    
    
    def drawVFHToVisualLog(self,histogram,candidateDirections,desiredDirection):
            
        #draw sectors                
        histogramSize = histogram.shape[0]
        angularSamplesPerDegree = histogramSize / 360
        sectorGap = 0    
        ringSize = int(2.1 * self.visualLog.getPixelsPerMeter())
        thickLineCompensation = 3#3#7#degrees
        res = [cv2.ellipse(self.visualLog.getImage(), \
                        (int(self.visualLog.getFrameWidth()/2) + self.visualLog.border,int(self.visualLog.getFrameHeight()/2) + self.visualLog.border),\
                        (ringSize,ringSize),\
                        0,\
                        #int(angle / angularSamplesPerDegree + math.degrees(- currentPose.getHeading() - HEADING_SHIFT - ANGULAR_RESOLUTION / 2) + sectorGap),\
                        #int(angle / angularSamplesPerDegree + math.degrees(- currentPose.getHeading() - HEADING_SHIFT + ANGULAR_RESOLUTION / 2) - sectorGap ),\
                        int(-(angle + thickLineCompensation) / angularSamplesPerDegree - HEADING_SHIFT_DEGREES + math.degrees(0.1) + sectorGap),\
                        int(-(angle + thickLineCompensation) / angularSamplesPerDegree - HEADING_SHIFT_DEGREES + math.degrees(-0.1) - sectorGap ),\
                        (0,0,255) if value else (0,255,0),\
                        3) for angle,value in enumerate(histogram)]
    
            
        ringSize = int(2.2 * self.visualLog.getPixelsPerMeter())
        color = (64,128,255)
        sectorGap = 2    
        image = self.visualLog.getImage()
        centerX = self.visualLog.getFrameWidth()/2
        centerY = self.visualLog.getFrameHeight()/2
        
        res = [cv2.ellipse(image, \
                        (int(centerX) + self.visualLog.border,int(centerY) + self.visualLog.border),\
                        (int(ringSize),int(ringSize) ),\
                        0,\
                        int(-math.degrees(candidate.getDirection()) - HEADING_SHIFT_DEGREES + sectorGap),\
                        int(-math.degrees(candidate.getDirection()) - HEADING_SHIFT_DEGREES - sectorGap ),\
                        color,\
                        3) for candidate in candidateDirections]
        
        ringSize = int(2.3 * self.visualLog.getPixelsPerMeter())
        sectorGap = 2   
        if desiredDirection:
            cv2.ellipse(self.visualLog.getImage(), \
                        (int(centerX) + self.visualLog.border,int(centerY) + self.visualLog.border),\
                        (int(ringSize),int(ringSize)),\
                        0,\
                        int(-math.degrees(desiredDirection) - HEADING_SHIFT_DEGREES + sectorGap),\
                        int(-math.degrees(desiredDirection) - HEADING_SHIFT_DEGREES - sectorGap ),\
                        #int(-math.degrees(candidate.getDirection()) - HEADING_SHIFT_DEGREES + sectorGap),\
                        #int(-math.degrees(candidate.getDirection()) - HEADING_SHIFT_DEGREES - sectorGap ),\
                        (255,255,0),\
                        3)
            
    
    
    def findOpenings(self,histogram):
        data = np.where(histogram-1)[0]
        rawOpenings = np.split(data, np.where(np.diff(data) != 1)[0]+1)
        openings = []
        histogramSize = histogram.shape[0]
        angularSamplesPerDegree = histogramSize / 360
        for rawOpening in rawOpenings:
            if len(rawOpening) == 0:
                #empty opening 
                continue
            opening = Opening(math.radians(rawOpening[0] / angularSamplesPerDegree),math.radians(rawOpening[-1] / angularSamplesPerDegree))
            openings.append(opening)                     
        
        #check if last opening is open -> check if it does not continue
        if len(openings) > 0 and openings[0].getRightBorder() == 0 and openings[-1].getLeftBorder() > math.radians(358):
            openings[0].setRightBorder(openings[-1].getRightBorder())
            openings.pop(-1)
            
        if len(openings) == 0:
            #no opening found -> everything is open or closed
            if not histogram[0]:
                opening = Opening(0,2*math.pi)
                openings.append(opening)
        return openings
        
    
            
    def findCandidateDirections(self,openings,directionOfWaypoint):
        #WIDE_OPENING_GRAVITY = 0
        #find candidate directions
        candidates = []
        isWide = False
        
        for opening in openings:
            if abs(mathex.distAngles(opening.getRightBorder(),opening.getLeftBorder())) > OPENING_THRESHOLD or (opening.getRightBorder() == 0 and opening.getLeftBorder() == math.pi * 2):
                #opening is wide
                candidate = Sector(opening.getRightBorder() + WIDE_OPENING_GRAVITY)  #was - 
                candidates.append(candidate)                         
                candidate = Sector(opening.getLeftBorder() - WIDE_OPENING_GRAVITY)  #was +
                candidates.append(candidate)
                isWide = True                         
                #is the target direction also candidate in this opening?
                #if robomath.distAngles(directionOfWaypoint,opening.getRightBorder())>= 0 and \
                #   robomath.distAngles(directionOfWaypoint,opening.getLeftBorder())<= 0:
                if mathex.isAngleBetween(directionOfWaypoint,opening.getRightBorder() + WIDE_OPENING_GRAVITY,opening.getLeftBorder() - WIDE_OPENING_GRAVITY): 
                   candidate = Sector(directionOfWaypoint)
                   candidates.append(candidate)
                if opening.getRightBorder() == 0 and opening.getLeftBorder() > math.radians(358):
                   #special case of complete opening                                  
                   candidate = Sector(directionOfWaypoint)
                   candidates.append(candidate)
            else:
                #opening is narrow
                #candidate direction is middle of the opening
                candidate = Sector(mathex.getMiddleAngle(opening.getRightBorder(),opening.getLeftBorder()))
                candidates.append(candidate)                         
        
        return candidates,isWide        
    
    def getBestDirection(self,candidateDirections,directionOfWaypoint):
        #calculate cost function
        if len(candidateDirections) == 0:
            return None
        minCandidate = candidateDirections[0]
        minCandidate.setCost(minCandidate.calculateCost(directionOfWaypoint,self.lastDirection))

        for candidate in candidateDirections:
            cost = candidate.calculateCost(directionOfWaypoint,self.lastDirection)
            candidate.setCost(cost)
            if minCandidate.getCost() > cost:
                candidate.setCost(cost)
                minCandidate = candidate             
        
        self.lastDirection = minCandidate.getDirection() 

        return minCandidate.getDirection()
    
class Opening:
    def __init__(self,rightBorder,leftBorder):
        self.leftBorder = leftBorder
        self.rightBorder = rightBorder

    def getLeftBorder(self):
        return self.leftBorder
    
    def getRightBorder(self):
        return self.rightBorder
        
    def setLeftBorder(self,value):
        self.leftBorder = value
    
    def setRightBorder(self,value):
        self.rightBorder = value

class Sector:
    def __init__(self,direction):
        self.direction = direction
        self.obstacleDensity = 0
        self.blocked = False
        self.cost = 0
        self.sunRayLimitDist = 999
        self.sunRayLimitPose = None                          

        
    def getDirection(self):
        return self.direction
    
    def setObstacleDensity(self,value):
        self.obstacleDensity = value
    
    def getObstacleDensity(self):
        return self.obstacleDensity

    def increaseObstacleDensity(self,value):
        self.obstacleDensity += value
    

    def setBlocked(self,value):
        self.blocked = value
    
    def getBlocked(self):
        return self.blocked
    
    def calculateCost(self,directionOfWaypoint,lastDirection):
       
        directionOfWaypointPart = DIRECTION_OF_WAYPOINT_FACTOR * min(abs(mathex.distAngles(directionOfWaypoint,self.direction)),abs(mathex.distAngles(self.direction,directionOfWaypoint)))
        currentDirectionPart =        CURRENT_DIRECTION_FACTOR * min(abs(mathex.distAngles(0,self.direction)),abs(mathex.distAngles(self.direction,0)))
        lastDirectionPart =              LAST_DIRECTION_FACTOR * min(abs(mathex.distAngles(lastDirection,self.direction)),abs(mathex.distAngles(self.direction,lastDirection)))
        return directionOfWaypointPart + currentDirectionPart + lastDirectionPart
        
    def getCost(self):
        return self.cost
    
    def setCost(self,value):
        self.cost = value

