#!/usr/bin/python
"""
  Driver for robots with support setSpeedPxPa().
"""

import math
from robot import angleDeg  # TODO move to some utils.py (?)
from line import *
from pose import normalizeAnglePIPI

def angleTo( f, t ):
  if math.fabs(f[0]-t[0]) < 0.0001 and math.fabs(f[1]-t[1]) < 0.0001:
    return 0
  return math.atan2( t[1]-f[1], t[0]-f[0] )

class Driver:
  def __init__( self, robot, maxSpeed = 1.0, maxAngularSpeed = 2*math.pi ):
    # note, that default max limits should be defined by min(Robot,Driver)
    self.robot = robot
    self.maxSpeed = maxSpeed
    self.maxAngularSpeed = maxAngularSpeed
    self.centerOffset = 0 # now only for followLineG hacking

  def stopDistance( self ):
    time = math.fabs(self.robot.currentSpeed/self.robot.maxAcc)+0.15 # system delay
    return 0.5 * self.robot.maxAcc * time * time

  def stopRotationAngle( self ):
    time = math.fabs(self.robot.currentAngularSpeed/self.robot.maxAngularAcc)+0.15 # system delay
    return 0.5 * self.robot.maxAngularAcc * time * time

  def restrictedTurn( self, turn):
    ''' Restricts the turn speed to [-maxAngularSpeed, +maxAngularSpeed]. '''
    if turn > self.maxAngularSpeed:
      return self.maxAngularSpeed
    elif turn < -self.maxAngularSpeed:
      return -self.maxAngularSpeed
    else:
      return turn

  def restrictedSpeed( self, turn):
    '''Restrict the speed with respect to the given rotational velocity.

    When turning fast, the forward velocity should be low.
    '''
    return self.maxSpeed * (0.5 + 0.5 *(1.0 - min(1.0, abs(turn / self.maxAngularSpeed))))

  def stopG( self, verbose=False ):
    if verbose:
      print "---- Driver.stop ----"
    for i in range(40): # TODO better avoidance of infinite loop
      yield ( 0.0, 0.0 )
      if math.fabs(self.robot.currentSpeed) < 0.01 and math.fabs(self.robot.currentAngularSpeed) < angleDeg(5):
        break
      
  def stop( self ):
    for cmd in self.stopG():
      self.robot.setSpeedPxPa( *cmd )
      self.robot.update()


  def wait(self, duration):
    ''' Wait for the given amount of time, having the robot stopped. '''
    self.robot.setSpeedPxPa(0, 0)
    startTime = self.robot.time
    while self.robot.time < startTime + duration:
      self.robot.update()  


  def goStraightG( self, dist, speed = None, withStop=True, verbose=False ):
    if verbose:
      print "---- Driver.goStraight(%.2f) ----" % dist
    if speed is None:
      speed = self.maxSpeed
    if dist >= 0:
      while dist > self.stopDistance():
        yield ( speed, 0.0 )
        dist -= self.robot.lastDistStep
    else:
      while dist < -self.stopDistance():
        yield ( -speed, 0.0 )
        dist -= self.robot.lastDistStep
    if withStop:
      for cmd in self.stopG():
        yield cmd

  def goStraight( self, dist, speed = None, withStop=True, timeout=None ):
    startTime = self.robot.time
    for cmd in self.goStraightG( dist, speed, withStop ):
      if timeout != None:
        if self.robot.time > startTime + timeout:
          return False
      self.robot.setSpeedPxPa( *cmd )
      self.robot.update()
    return True


  def turnG( self, angle, angularSpeed = None, radius = 0.0, angleThreshold = math.radians(10), withStop=True, verbose=False ):
    """ Generator - turn in place (could be even several whole circles) 
      - caller has to normalize angle if necessary
      - radius is positive for forward turn"""
    if verbose:
      if radius != 0:
        print "---- Driver.turn(%d, rad=%.2f) ----" % (int(math.degrees(angle)), radius)
      else:
        print "---- Driver.turn(%d) ----" % int(math.degrees(angle))
    if angularSpeed is None:
      angularSpeed = self.maxAngularSpeed
    else:
      angularSpeed = self.restrictedTurn(angularSpeed)
    if math.fabs( angle ) < angleThreshold: # well, there is some minimal limit but ...
      return
    if angle < 0:
      while angle < -self.stopRotationAngle():
        yield ( radius*angularSpeed, -angularSpeed )
        angle -= self.robot.lastAngleStep
    else:
      while angle > self.stopRotationAngle():
        yield ( radius*angularSpeed, angularSpeed )
        angle -= self.robot.lastAngleStep
    if withStop:
      for cmd in self.stopG():
        yield cmd
        angle -= self.robot.lastAngleStep
    if verbose:
      print "---- Driver.turn result(%d) ----" % int(math.degrees(angle))

  def turn( self, angle, angularSpeed = None, radius = 0.0, timeout = None, withStop=True, verbose=False ):
    startTime = self.robot.time
    for cmd in self.turnG( angle, angularSpeed, radius = radius, withStop=withStop, verbose=verbose ):
      if timeout != None:
        if self.robot.time > startTime + timeout:
          return False
      self.robot.setSpeedPxPa( *cmd )
      self.robot.update()
    return True


  def followLineG( self, line, stopDistance = 0.0, turnScale = 4.0, offsetSpeed = math.radians(20), offsetDistance = 0.03 ):
    '''experimental generator - to replace orig function

    line           ... A line to follow.
    stopDistance   ... The robot stops when closer than this to the endpoint. [m]
    turnScale      ... Magic parameter for the rotational speed. [scaling factor]
    offsetSpeed    ... This extra rotational speed is added when the robot is too far from the line. [rad/s]
    offsetDistance ... When the robot is further than this from the line, some extra correction may be needed. [m]
    '''
    while line.distanceToFinishLine( self.robot.localisation.pose() ) > stopDistance:
      diff = normalizeAnglePIPI( line.angle - self.robot.localisation.pose()[2] );
      signedDistance = line.signedDistance( self.robot.localisation.pose() ) + self.centerOffset
#      print "deg %.1f" %( math.degrees(diff),), "dist=%0.3f" % (signedDistance,)
      if math.fabs( signedDistance ) > offsetDistance:
        step = max(0.0, min(offsetSpeed, offsetSpeed * (abs(signedDistance)-offsetDistance)/offsetDistance ))
        if signedDistance < 0:
          diff += step
        else:
          diff -= step
      turn = self.restrictedTurn(turnScale * diff)
      speed = self.restrictedSpeed(turn)
      yield  speed, turn

  def followLine( self, line, stopDistance = 0.0 ):
    "deprecated - there will be support of generators only"
    for cmd in self.followLineG( line, stopDistance ):
      self.robot.setSpeedPxPa( *cmd )
      self.robot.update()


  def followPolyLineG( self, pts, stopDistance = 0.1, angleThreshold = math.radians(20), turnScale = 4.0, offsetSpeed = math.radians(20), offsetDistance = 0.03, withStops=False ):
    for a,b in zip(pts[:-1],pts[1:]):
      print "--- follow (%0.2f,%0.2f) -> (%0.2f,%0.2f) ---" % ( a[0], a[1], b[0], b[1] )
      pose = self.robot.localisation.pose()
      angleDiff = normalizeAnglePIPI( angleTo(pose, b) - pose[2])
      if math.fabs( angleDiff ) > angleThreshold:
        for cmd in self.turnG( angleDiff, angleThreshold = angleThreshold ):
          yield cmd        
      line = Line(a,b)
      for cmd in self.followLineG( line, stopDistance = stopDistance, turnScale = turnScale, offsetSpeed = offsetSpeed, offsetDistance = offsetDistance ):
        yield cmd
      if withStops:
        for cmd in self.stopG():
          yield cmd
    for cmd in self.stopG():
      yield cmd

  def followPolyLine( self, pts ):
    for cmd in self.followPolyLineG( pts ):
      self.robot.setSpeedPxPa( *cmd )
      self.robot.update()


  def cmdDaisy( self, pose, goal, maxSpeed ):
    angleDiff = normalizeAnglePIPI( angleTo(pose, goal)-pose[2] )
    angularSpeed = self.maxAngularSpeed
    if angleDiff > math.pi/2.0:
      return ( 0.0, angularSpeed )
    elif angleDiff < -math.pi/2.0:
      return ( 0.0, -angularSpeed )
    return math.cos(angleDiff)*maxSpeed, math.sin(angleDiff)*angularSpeed

  def followDaisyRouteG( self, route, prediction ):
    while True:
      pose = self.robot.localisation.pose()
      before, future = route.routeSplit( pose )
      route.pts = future
      if route.length() < prediction:
        break
      goal = route.pointAtDist( prediction )
      if distance( pose, goal ) < 0.01:
        goal = route.pointAtDist( 2*prediction )
      futureGoal = route.pointAtDist( 3*prediction )
      scale = max(distance( pose, futureGoal )/(3*prediction), 0.1) # the robot has to be able to move away
      yield self.cmdDaisy( pose, goal, self.maxSpeed*scale )


  def goToG( self, target, finishRadius, backward=False, angleThreshold = math.radians(10), angularSpeed = math.radians(10) ):
    "generator to reach given destination with given precision"
    headingOffset = backward and math.radians(180) or 0.0
    prevPose = pose = self.robot.localisation.pose()
    angularSpeed = self.restrictedTurn(angularSpeed)
    while distance( pose, target ) > finishRadius:
      angleDiff = normalizeAnglePIPI( angleTo(pose, target) - pose[2] - headingOffset)
#      print angleDiff
      if math.fabs( angleDiff ) > angleThreshold:
        for cmd in self.turnG( angleDiff, angleThreshold = angleThreshold ):
          yield cmd        
      pose = self.robot.localisation.pose()
      if distance( pose, target ) < self.stopDistance():
        break

      angleDiff = normalizeAnglePIPI( angleTo(pose, target) - pose[2] - headingOffset)
      if math.fabs( angleDiff ) > angleThreshold / 2.0:
        turn = angularSpeed if angleDiff > 0 else -angularSpeed
      else:
        turn = math.fabs(angleDiff / angleThreshold / 2.0) * (angularSpeed if angleDiff > 0 else -angularSpeed)
      speed = self.restrictedSpeed(turn)
      if backward:
        speed *= -1
      yield ( speed, turn )

    for cmd in self.stopG():
      yield cmd

  def goTo( self, target, finishRadius, backward=False ):
    for cmd in self.goToG( target, finishRadius, backward ):
      self.robot.setSpeedPxPa( *cmd )
      self.robot.update()

  def spiralG( self, step, speed = None ):
    "generate infinite spiral"
    if speed is None:
      speed = self.maxSpeed
    radius = step
    while 1:
      gen = self.turnG( math.radians(180.0), angularSpeed=abs(speed/radius), radius=radius, withStop=False )
      for cmd in gen:
        yield cmd
      radius += step/2.0 # semicircles

  def multiGen( self, listOfGenerators ):
    for gen in listOfGenerators:
      for cmd in gen:
        yield cmd

