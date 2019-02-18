#!/usr/bin/env python

from itertools import islice, izip
import math

FAR_AWAY = 256.0  # A dummy far-away distance used instead of zeros in laser scans. [m]

class VFH:
  '''
  VFH+ implementation. No memory is used, only the obstacles in the last laser scan are taken in account.

  The laser is expected to be mounted on the axis in the robot's facing direction.

  Modeled after "VFH+: Reliable Obstacle Avoidance for Fast Mobile Robots", I. Ulrich and J. Borenstein.
  '''
  def __init__( self,
                robot,
                laserFOV = math.radians(270),
                blockingDistance = 0.35,
                safetyDistance = 0.6,
                maxRange = 1.4,
                turnRadius = None,
                mu1 = 5,
                mu2 = 2,
                mu3 = 3,
                binSize = math.radians(5),
                laserDensity = 1,
                toleratedLaserFailures = 3,
                verbose = False,
                sensorID = "laser"):
    '''
    Params:
      robot            ... The robot.
      laserFOV         ... Field of view of the laser. [rad]
      safetyDistance   ... This is a minimal planned distance from obstacles. [m]
      maxRange         ... Reading further than this threshold are ignored. This makes it possible to use a laser pointing to the ground. [m]
      turnRadius       ... The maximal turning trajectory of the robot is approximated with a circle with this radius. If None, a holonomic robot is assumed. [m]
      mu1, mu2, mu3    ... Parameters of the cost function. mu1 is a weight of a difference between a candidate direction annd the goal direction; mu2 is a weight of a difference between a candidate direction and the current steering direction; mu3 is a weight of difference between a candidate direction and the previously selected direction. mu1 > mu2 + mu3 must hold. TODO: mu2 is currently not used.
      binSize          ... Angular width of the bins in the histograms. laserFOV should be divisible by binSize. [rad]
      laserDensity     ... Only every laserDensity-th beam is taken into account.
      toleratedLaserFailures ... Number of consecutive "no-data" by laser, which is still considered to be ok.
      verbose          ... If verbose is set to True, some debugging information may be printed.
    '''
    self.robot = robot
    self.laserMeasurements = None # Laser measurements. List of angle+distance pairs.
    self.laserFOV = laserFOV
    self.blockingDistance = blockingDistance
    self.safetyDistance = safetyDistance
    self.maxRange = maxRange
    self.turnRadius = turnRadius
    self.mu1 = mu1
    self.mu2 = mu2
    self.mu3 = mu3
    self.binSize = binSize
    self.laserDensity = laserDensity
    self.toleratedLaserFailures = toleratedLaserFailures
    self.verbose = verbose
    self.sensorID = sensorID

    self.__laserFailures = 0

  def __bin(self, angle):
    ''' Find an index of a bin which the angle falls to. '''
    return int( (angle + self.laserFOV / 2.0) / self.binSize)

  def __angle(self, bin):
    ''' Find an angle corresponding to the given bin. '''
    return bin * self.binSize + self.binSize / 2.0 - self.laserFOV / 2.0

  def __obstDir(self, i, n):
    ''' Compute a direction to the obstacle from the scan index.

    i ... index of the scan
    n ... total amount of scan values
    '''
    return self.laserFOV * i / (n - 1.0) - self.laserFOV / 2.0

  def updateExtension(self, robot, id, data):
    if id == self.sensorID:
      n = len(data)
      self.laserMeasurements = [(self.__obstDir(i, n), d / 1000.)
                                for (i, d) in islice( enumerate(data),
                                                      0,
                                                      len(data),
                                                      self.laserDensity)
                                if d > 0]

      if not data:
        self.__laserFailures += 1
      else:
        self.__laserFailures = 0

  def isBlocked(self, extraObs = []):
    '''Return True if there is an obstacle within self.safetyDistance in front of the robot.

       extraObs ... A list of extra obstacles. An obstacle is specified in local polar coordinates as a pair (angle, distance) [(rad, m)].
    '''
    if self.__laserFailures > self.toleratedLaserFailures:
      return True
    if self.laserMeasurements is None:
      return True # Not initialized + better safe than sorry
    def threshold(angle):
      # A non-circular blocking shape
      return self.blockingDistance * (0.7 + 0.3 * math.cos(angle))
    return (any(x < threshold(a) for (a, x) in self.laserMeasurements)
            or any(d < threshold(a) for (a, d) in extraObs))

  def navigate(self, goalDir, prevDir = 0.0, extraObs = []):
    ''' Find a direction to the goal avoiding obstacles visible in the last scan.

    Param:
       goalDir ... A direction to the goal. [rad]
       prevDir ... A previously selected steering direction. [rad]
       extraObs ... A list of extra obstacles. An obstacle is specified in local polar coordinates as a pair (angle, distance) [(rad, m)].
    Return:
       Either a preferred direction [rad, counter-clockwise], or None.
       None is returned whenever there is an obstacle very near or no direction seems reasonable.
    '''
    if self.laserMeasurements is None:
      return None

    if self.isBlocked(extraObs):
      return None

    polarHistogram = self.__polarHistogram(extraObs)
    if polarHistogram is None:
      return None
    binaryPolarHistogram = self.__binaryPolarHistogram(polarHistogram)
    maskedPolarHistogram = self.__maskedPolarHistogram(binaryPolarHistogram)
    openWindows = self.__openWindows(maskedPolarHistogram)
    candidates = self.__candidates(openWindows, goalDir)
    dir = self.__bestCandidate(candidates, goalDir, prevDir)
    if self.verbose:
      s = [ 'x' if x else ' ' for x in maskedPolarHistogram]
      if dir is not None:
        i = self.__bin(dir)
        s = s[:i] + ['.'] + s[i:]
      s = ''.join(s)
      print "'" + s[::-1] + "'"
    return dir

  def __polarHistogram(self, extraObs = []):
    # In The Article, the threshold is carefully computed from unspecified parameters.
    # In this implementation, the threshold denotes a maximal tolerated number of short reading within a bin.
    self.__histogramThreshold = 0.

    polarHistogram = [0.0] * (1 + self.__bin(self.laserFOV / 2.0))
    obstacles = [ (beta, d) for (beta, d) in extraObs if d <= self.maxRange ]
    for measurement in self.laserMeasurements:
      d = measurement[1] # distance [m]
      if d > self.maxRange:
        continue
      obstacles.append(measurement)

    for (beta, d) in obstacles:
      m = 1. #In the Article, this is a carefully computed cell magnitude. With the laser scanner, we can take any obstacle within the range as a road block.
      if d < self.blockingDistance: # we are within the safety distance from an obstacle
        return None
      ratio = self.safetyDistance / d
      # if we are within the safetyDistance, asin is undefined => let's HACK
      if ratio > 1.0:
        ratio = 1.0
      elif ratio < -1.0:
        ratio = -1.0
      gamma = math.asin(ratio) # enlargement angle [rad]

      low = max(0, self.__bin( beta - gamma ))
      high = min(len(polarHistogram), self.__bin( beta + gamma ))
      for j in range(low, high):
        polarHistogram[j] += m

    return polarHistogram

  def __binaryPolarHistogram(self, polarHistogram):
    return [ x > self.__histogramThreshold for x in polarHistogram ] #NOTE: No hysteresis. (Unlike in the article)

  def __maskedPolarHistogram(self, binaryPolarHistogram):
    if self.turnRadius is None: # A holonomic robot.
      return binaryPolarHistogram 
    else:
      MARGIN = math.pi
      left = +self.laserFOV / 2.0 + MARGIN
      right = -self.laserFOV / 2.0 - MARGIN

      # Centers of the turning circles in the local coordinate frame [m, m]
      (rcx, rcy) = (0., -self.turnRadius) # Center of the right-turning circle.
      (lcx, lcy) = (0., self.turnRadius) # Center of the left-turning circle.

      def isLeftFrom(phiWho, phiFrom):
        ''' Is the phiWho angle left from the phiFrom angle? '''
        return phiWho > phiFrom
      def isRightFrom(phiWho, phiFrom):
        ''' Is the phiWho angle right from the phiFrom angle? '''
        return phiWho < phiFrom

      for (beta, dist) in self.laserMeasurements:
        if dist > self.maxRange:
          continue

        (obst_x, obst_y) = (dist * math.cos(beta), dist * math.sin(beta)) # Location of the obstacle in the local coordinate frame. [m, m]

        if isLeftFrom(beta, right): # Right-turning to this direction and beyond is not blocked yet.
          d = math.hypot(obst_x - rcx, obst_y - rcy)
          if d < self.turnRadius + self.safetyDistance:
            right = beta # An obstacle blocking turning to the right.
        if isRightFrom(beta, left):
          d = math.hypot(obst_x - lcx, obst_y - lcy)
          if d < self.turnRadius + self.safetyDistance:
            left = beta

      li = self.__bin(left)
      ri = self.__bin(right)

      return [ False if binaryPolarHistogram[i] == False and i >= ri and i <= li else True for i in xrange(len(binaryPolarHistogram)) ]

  def __openWindows(self, maskedPolarHistogram):
    openWindows = []
    prev = True

    for i in xrange(1 + len(maskedPolarHistogram)):
      mask = True if i == len(maskedPolarHistogram) else maskedPolarHistogram[i]
      if prev == True and mask == False: # Right edge of a window.
        right = self.__angle(i)

      if prev == False and mask == True: # Left edge of a window.
        left = self.__angle(i - 1)
        openWindows.append( (right,left) )

      prev = mask

    return openWindows

  def __candidates(self, openWindows, goalDir):
    candidates = []

    for (right, left) in openWindows:
      if goalDir >= right and goalDir <= left:
        candidates.append(goalDir)

      # Note: Not distinguishing wide and narrow openings as in The Article.
      candidates.append(right)
      candidates.append(left)

    return candidates

  def __bestCandidate(self, candidates, goalDir, prevDir):
    bestDir = None
    bestCost = None

    for dir in candidates:
      cost = self.mu1 * abs(dir - goalDir) + self.mu3 * abs(dir - prevDir) #TODO: mu2
      if bestDir is None or cost < bestCost:
        bestDir = dir
        bestCost = cost

    return bestDir

  def __remask(self, mask, n):
    ''' Rescales the mask to the given length.'''
    m = len(mask)

    if m == n:
      return mask # no need to do anything

    return [ mask[int(round(i * (m - 1) / float(n - 1)))] for i in xrange(n) ]

# Imports needed for the test
from driver import Driver, normalizeAnglePIPI
from eduro import EmergencyStopException
from localisation import SimpleOdometry
from ray_trace import combinedPose
class VFHTest:
  def __init__(self, robot, dummy = None, verbose = False):
    self.robot = robot
    self.verbose = verbose

    self.robot.attachLaser()
    self.robot.attachEmergencyStopButton()

    self.robot.laser.stopOnExit = False  # for faster boot-up

  def __call__(self):
    try:
      if getattr( self.robot.laser, 'startLaser', None ):
        # trigger rotation of the laser, low level function, ignore for log files
        print "Powering laser ON"
        self.robot.laser.startLaser() 

      self.robot.waitForStart()
      self.robot.laser.start()  # laser also after start -- it should be already running
      self.robot.localisation = SimpleOdometry()
      self.driver = Driver( self.robot,
                            maxSpeed = 0.5,
                            maxAngularSpeed = math.radians(180))

      self.testVFH(verbose = self.verbose)
    except EmergencyStopException, e:
      print "EmergencyStopException"

    self.robot.gps.requestStop()
    self.robot.laser.requestStop()
    self.robot.camera.requestStop()

  def testVFH(self, verbose = False):
    turnRadius = None #1.2
    vfh = VFH(self.robot,
              blockingDistance = 0.35,
              safetyDistance = 0.6,
              maxRange = 1.4,
              turnRadius = turnRadius,
              verbose = verbose)
    self.robot.addExtension(vfh.updateExtension)

    TOLERATED_MISS = 0.2 # [m]
    ANGULAR_THRESHOLD = math.radians(20) # [rad]
    ANGULAR_SPEED = math.radians(60) # [rad/s]
    D = 20.0 # [m]
    waypoints = [ (D, 0.0), (D, D), (0.0, D), (0.0, 0.0) ]
    prevDir = 0.0 # [rad]

    while True:
      for (x, y) in waypoints:
        isThere = False
        while not isThere:
          pose = self.robot.localisation.pose()
          if vfh.laserMeasurements is None:
            strategy = self.driver.stopG()
            prevDir = 0.0
          else:
            phi = math.atan2(y - pose[1], x- pose[0])
            goalDir = normalizeAnglePIPI(phi - pose[2])
            dir = vfh.navigate(goalDir, prevDir)

            if dir is None:
              strategy = self.driver.stopG()
              prevDir= 0.0
            else:
              goal = combinedPose((pose[0], pose[1], pose[2]+dir), (1.0, 0, 0))
              strategy = self.driver.goToG( goal,
                                            TOLERATED_MISS,
                                            angleThreshold = ANGULAR_THRESHOLD,
                                            angularSpeed = ANGULAR_SPEED)
              prevDir = dir

            vfh.laserMeasurements = None

          for (speed, angularSpeed) in strategy:
            self.robot.setSpeedPxPa( speed, angularSpeed )
            self.robot.update()

            pose = self.robot.localisation.pose()
            isThere = math.hypot(pose[0] - x, pose[1] - y) <= TOLERATED_MISS
            if isThere or vfh.laserMeasurements is not None:
              break

if __name__ == "__main__":
  import sys
  from eduromaxi import EduroMaxi
  import launcher
  launcher.launch(sys.argv, EduroMaxi, VFHTest)

