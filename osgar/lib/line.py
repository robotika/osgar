#!/usr/bin/python
"""
  Geometry line helper based on Zefyros project.
"""

import math

def distance( planar1, planar2 ):
  "distane two planar points"
  x = planar1[0] - planar2[0]
  y = planar1[1] - planar2[1]
  return math.hypot(x, y)

def pointAtPolyLineDist( pts, dist ):
 # return point on polyline (planar) in given distance
  if len(pts) == 0:
    return None
  if len(pts) == 1:
    return pts[0]

  prev = pts[0]
  distFromStart = 0
  for p in pts[1:]:
    distFromStart += distance( prev, p )
    if dist < distFromStart:
      break
    prev = p
  if distance( prev, p ) < 0.000001:
    return p
  # resize (p - prev) to dist-distFromStart
  assert( dist < distFromStart )
  frac = (distFromStart - dist)/distance( prev, p )
  return ( p[0] + frac*(prev[0]-p[0]), p[1] + frac*(prev[1]-p[1]) ) 


class Line:

  def __init__( self, start, end ):
    self.start = start
    self.end = end
    self.angle = math.atan2( end[1] - start[1], end[0] - start[0] )
    self._a = -math.sin( self.angle )
    self._b = math.cos( self.angle )
    self._c = -self._a * start[0] - self._b * start[1]
    self._perpC = self._b * end[0] - self._a * end[1] # perpendicular line in end position
    self.length = distance( start, end )

  def signedDistance( self, pos ):
    return self._a * pos[0] + self._b * pos[1] + self._c

  def distanceToFinishLine( self, pos ):
    return - self._b * pos[0] + self._a * pos[1] + self._perpC;

  def finished( self, pos ):
    return self.distanceToFinishLine( pos ) <= 0

  def snap( self, pos ):
    dist = self.distanceToFinishLine( pos )
    return (self.end[0] - self._b * dist, self.end[1] + self._a * dist )

  def nearest( self, pos ):
    "return abs distance and type 0=start, 1=end, -1=inside of line"
    dist = self.distanceToFinishLine( pos )
    if dist <= 0:
      return self.end, distance(pos,self.end), 1
    if dist >= self.length:
      return self.start, distance(pos,self.start), 0
    return self.snap(pos), math.fabs(self.signedDistance(pos)), -1

