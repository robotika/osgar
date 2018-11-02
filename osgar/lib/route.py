"""
  Route with geo2plan and plan2geo conversions + snap position.
"""

import math
from osgar.lib.line import Line, distance, pointAtPolyLineDist


EARTHRADIUS = 6.380935276e+06
SUIDARHTRAE = 1.567168380e-07
DEGRAD            = 1.7453292519943295769236907684886e-02
RADDEG            = 5.7295779513082320876798154814105e+01


def loadLatLonPtsFromStr(input_str):
    lines = input_str.split("\n")
    pts = []
    for line in lines:
        if len(line) > 0 and line[0] != '#':
            assert( len(line.split()) == 2 )
            lat, lon = line.split()
            pts.append( (float(lon), float(lat)) )
    return pts


def loadLatLonPts(filename):
    with open(filename, "r") as f:
        return loadLatLonPtsFromStr(f.read())


def saveLatLonPts(pts, filename, note = None):
    with open(filename, "w") as f:
        if note:
            f.write("# " + note + "\n")
        for p in pts:
            f.write("%f %f\n" % (p[1], p[0]))


class Convertor:
    "convert lat/lon and planar coordinates"
    def __init__(self, refPoint = (16.60906, 49.2060633333)):
        self._cosMulti = max(0.001, EARTHRADIUS*math.cos(refPoint[1]*DEGRAD));
        self.refPoint = refPoint

    def geo2planar(self, pos):
        return ((pos[0] - self.refPoint[0]) * DEGRAD * self._cosMulti, (pos[1] - self.refPoint[1]) * DEGRAD * EARTHRADIUS)

    def planar2geo(self, pos):
        return (self.refPoint[0] + pos[0] * RADDEG/ self._cosMulti, self.refPoint[1] + pos[1] * RADDEG * SUIDARHTRAE)


class DummyConvertor:
    "convert 1:1 for coordinates already in meters"
    def geo2planar(self, pos):
        return pos

    def planar2geo(self, pos):
        return pos


class Route:
    """ route for robot navigation"""
    def __init__(self, pts = [], conv = None, isLoop = None):
        if conv == None:
            conv = Convertor()
        self.conv = conv
        self.pts = [ self.conv.geo2planar(p) for p in pts ]
        if isLoop == None and len(self.pts) > 1:
            isLoop = (distance(self.pts[0], self.pts[-1]) < 5.0)
        self.isLoop = isLoop

    def length( self ):
        if not self.pts:
            return None
        sum = 0
        prev = self.pts[0]
        for p in self.pts:
            sum += distance( prev, p )
            prev = p
        return sum

    def findNearestEx( self, pos ):
        """
        Find nearest position on the polyline and return tuple:
           - nearest point(x,y) on route
           - distance to that point
           - index of segment (negative) or waypoint index (0..N-1)
        """
        if not self.pts:
            return None
        ref = self.conv.geo2planar( pos )
        index = 0
        best = prev = self.pts[0]
        bestDist = distance( ref, best )
        bestIndex = 0
        for p in self.pts[1:]:
            pos, dist, type = Line(prev, p).nearest( ref )
            if dist < bestDist:
                bestDist = dist
                best = pos
                if type < 0:
                    bestIndex = -index -1
                else:
                    bestIndex = index + type
            prev = p
            index += 1
        return self.conv.planar2geo( best ), bestDist, bestIndex

    def findNearest( self, pos ):
        """ filtered version, returning only (x,y) coordinate of nearest point on route """
        near = self.findNearestEx( pos )
        if near:
            return near[0] # position
        return None

    def routeSplit( self, pos ):
        """
        Split path into two parts based on position `pos`:
          - already passed route
          - remaing part to be driven
        """
        snap, dist, index = self.findNearestEx( pos )
        if index < 0:
            # inside line -> split line
            first = [self.conv.planar2geo(p) for p in self.pts[:-index]] + [snap]
            second = [snap]+[self.conv.planar2geo(p) for p in self.pts[-index:]]
        else:
            # some end point
            first = [self.conv.planar2geo(p) for p in self.pts[:index+1]] 
            second = [self.conv.planar2geo(p) for p in self.pts[index:]]
        return first, second

    def pointAtDist( self, dist ):
        """ return point on route in given distance from the start """
        if len(self.pts) < 1:
            return None
        return self.conv.planar2geo( pointAtPolyLineDist( self.pts, dist ) )

    def turnAngleAt( self, pos, radius = 2.0 ):
        """
        Compute "derivation" at nearest point on the route. The radius
        defines how far previous and next point are placed. The returned
        angle is given as a difference from previus and next vectors.
        """
        first, second = self.routeSplit( pos )
        assert len(first) >= 1, len(first)
        assert len(second) >= 1, len(second)
        if self.isLoop:
            geoPts = [self.conv.planar2geo(x) for x in self.pts]
            first = geoPts + first
            second = second + geoPts
        if len(first) == 1 or len(second) == 1:
            return 0.0 # start or end of route
        nextPos = pointAtPolyLineDist( [ self.conv.geo2planar(x) for x in second ], radius )
        currPos = self.conv.geo2planar( second[0] )
        first.reverse()
        prevPos = pointAtPolyLineDist( [ self.conv.geo2planar(x) for x in first ], radius )
        toNext = math.atan2( nextPos[1]-currPos[1], nextPos[0]-currPos[0] )
        toPrev = math.atan2( currPos[1]-prevPos[1], currPos[0]-prevPos[0] )
        ret = toNext - toPrev
        if ret < -math.pi:
            ret += 2*math.pi
        if ret > math.pi:
            ret -= 2*math.pi
        return ret

# vim: expandtab sw=4 ts=4
