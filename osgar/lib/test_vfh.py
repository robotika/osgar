#!/usr/bin/env python

import math

from vfh import VFH

class PseudoRobot:
  pass

def createVFH(verbose=False):
  return VFH(robot, laserFOV=fov, blockingDistance=0., safetyDistance=0.10, maxRange=1.5, turnRadius=0.5, binSize=binSize, verbose=verbose)

if __name__ == '__main__':
  robot = PseudoRobot()
  FOV_DEGS = 270
  fov = math.radians(FOV_DEGS)
  binSize = math.radians(5)
  vfh = createVFH()
  scan = [0] * (FOV_DEGS * 2 + 1)

  # There is an obstacle slightly to the right from the straight direction ...
  (obstacle_x, obstacle_y) = (0. + 0.6 * math.cos(math.pi/4), -0.5 + 0.6 * math.sin(math.pi/4))
  obst_phi = math.atan2(obstacle_y, obstacle_x)
  i = int((obst_phi + fov / 2.)/math.radians(0.5))
  scan[i] = int(1000 * math.hypot(obstacle_x, obstacle_y))
  vfh.updateExtension(robot, 'laser', scan)

  # We must pass the obstacle to the left. a) Everything to the right is blocked due to the turnRadius, b) the straight direction is blocked due to the safetyDistance (the straight route passes too near)
  for gd in xrange(0, -45, -1):
    advice = vfh.navigate(goalDir=math.radians(gd))
    #print 'Advice:', math.degrees(advice)
    assert(advice is not None)
    assert(advice > 0)

  # Let's try the mirrored case.
  # Due to discretization and alignment within the bins, the output may not be symmetrical around zero.
  scan.reverse()
  vfh = createVFH() # A fresh VFH is not influenced by any hysteresis or filtering.
  vfh.updateExtension(robot, 'laser', scan)
  for gd in xrange(0, 45, +1):
    advice = vfh.navigate(goalDir=math.radians(gd))
    #print 'Advice:', math.degrees(advice)
    assert(advice is not None)
    assert(advice < 0) # We must pass the obstacle to the right. See the explanation above for the mirrored case.

  print 'OK'
