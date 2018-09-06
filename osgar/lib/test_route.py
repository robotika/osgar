import unittest
import math

from .route import *


class RouteTest( unittest.TestCase ):

    def assertEqualCoords( self, p, q, places=6 ):
        self.assertAlmostEqual( p[0], q[0], places )
        self.assertAlmostEqual( p[1], q[1], places )

    def testFlattenConversion( self ):
        conv = Convertor()
        geo = (16.00, 49.00)
        planar = conv.geo2planar( geo )
        geo2 = conv.planar2geo( planar )
        self.assertEqualCoords( geo, geo2 )

    def testFindNearest( self ):
        self.assertEqual( Route().findNearest( (16.0, 49.0) ), None )
        self.assertEqualCoords( Route([(17.0, 51.3)]).findNearest( (16.0, 49.0) ), (17.0, 51.3) )
        r = Route( [(10.0, 51.0), (20.0, 52.0), (30.0, 43.2)] )
        self.assertEqualCoords( r.findNearest( (22.0, 50.2) ), (22.029302716939391, 50.214213605948785) )

    def testFindNearestEx( self ):
        conv = Convertor( (0.0, 0.0) )
        r = Route( conv = conv )
        pts = []
        for i in range(10):
            pts.append( (i, 0.0) )
        for i in range(10):
            pts.append( (10.0, i) )
        r.pts = pts
        (pos, dist, index) = r.findNearestEx( conv.planar2geo( (2.5, 3.0) ) )
        self.assertAlmostEqual( 3.0, dist, 5 )
        self.assertEqual( -3, index )
        self.assertEqual( -1, r.findNearestEx( conv.planar2geo( (0.5, 3.0) ) )[2] )
        self.assertEqual( 0, r.findNearestEx( conv.planar2geo( (-0.5, 3.0) ) )[2] )
        self.assertEqual( 10, r.findNearestEx( conv.planar2geo( (20, -20) ) )[2] )

    def testRouteSplit( self ):
        conv = Convertor( (0.0, 0.0) )
        r = Route( conv = conv )
        pts = []
        for i in range(10):
            pts.append( (i, 0.0) )
        for i in range(11):
            pts.append( (10.0, i) )
        r.pts = pts
        first, second = r.routeSplit( conv.planar2geo( (20, -20) ) )
        self.assertEqual( 11, len(first) )
        self.assertEqual( 11, len(second) )
        self.assertEqualCoords( (10.0,0.0), conv.geo2planar( first[-1] ) )
        self.assertEqualCoords( (10.0,0.0), conv.geo2planar( second[0] ) )
        first, second = r.routeSplit( conv.planar2geo( (2.5, -2.0) ) )
        self.assertEqual( 4, len(first) ) # 0 1 2 2.5
        self.assertEqual( 21-2, len(second) ) # 2.5 3 4 ...

    def testPointAtDist( self ):
        conv = Convertor( (0.0, 0.0) )
        r = Route( conv = conv )
        self.assertEqual( None, r.pointAtDist( 2.5 ) )
        pts = []
        for i in range(10):
            pts.append( (i, 0.0) )
        r.pts = pts
        self.assertEqualCoords( (2.5,0.0), conv.geo2planar(r.pointAtDist( 2.5 )) )
        self.assertEqualCoords( (5.0,0.0), conv.geo2planar(r.pointAtDist( 5.0 )) )
        self.assertEqualCoords( (9.0,0.0), conv.geo2planar(r.pointAtDist( 500.0 )) )
        self.assertEqualCoords( (0.0,0.0), conv.geo2planar(r.pointAtDist( 0.0 )) )

    def testTurnAngleAt( self ):
        conv = Convertor( (0.0, 0.0) )
        r = Route( conv = conv )
        pts = []
        for i in range(10):
            pts.append( (i, 0.0) )
        for i in range(11):
            pts.append( (10.0, i) )
        for i in range(10):
            pts.append( (10.0+i, 10.0) )
        r.pts = pts
        self.assertAlmostEqual( 0.0, r.turnAngleAt( conv.planar2geo( (2, -2) ) ), 5 )
        self.assertAlmostEqual( math.pi/2, r.turnAngleAt( conv.planar2geo( (10, 0) ) ), 5 )
        self.assertAlmostEqual( -math.pi/2, r.turnAngleAt( conv.planar2geo( (10, 10) ) ), 5 )
        r.pts.reverse()
        self.assertAlmostEqual( 0.0, r.turnAngleAt( conv.planar2geo( (2, -2) ) ), 5 )
        self.assertAlmostEqual( -math.pi/2, r.turnAngleAt( conv.planar2geo( (10, 0) ) ), 5 )
        self.assertAlmostEqual( math.pi/2, r.turnAngleAt( conv.planar2geo( (10, 10) ) ), 5 )

        # behind EndOfRoute (problem detected during testing Malesice090917
        self.assertAlmostEqual( 0.0, r.turnAngleAt( conv.planar2geo( (-20, 0) ) ), 5 )
        self.assertAlmostEqual( 0.0, r.turnAngleAt( conv.planar2geo( (100, 100) ) ), 5 )
        self.assertFalse(r.isLoop)

    def testIsLoop( self ):
        conv = Convertor( (0.0, 0.0) )
        pts = []
        for i in range(10):
            pts.append( (i, 0.0) )
        for i in range(10):
            pts.append( (10.0, i) )
        for i in range(10):
            pts.append( (10.0-i, 10.0) )
        for i in range(10):
            pts.append( (0.0, 10.0-i) )

        r = Route( [conv.planar2geo(x) for x in pts], conv = conv )
        self.assertTrue( r.isLoop )
        self.assertAlmostEqual( math.pi/2, r.turnAngleAt( conv.planar2geo( (-1, -1) ) ), 5 )

    def testMalesice090917LastRound( self ):
        tomasOkruh = """
# Converted from TomasOkruh.kml
50.084366 14.498719
50.084271 14.498445
50.084502 14.498133
50.084654 14.497916
50.084673 14.497982
50.084676 14.498478
50.084708 14.498862
50.085279 14.498498
50.085586 14.498983
50.085407 14.499124
50.085356 14.499679 
"""
        r = Route( loadLatLonPtsFromStr( tomasOkruh ) )
        self.assertTrue( r.length() < 1000 )
        self.assertTrue( r.findNearestEx( (14.498925, 50.084739999999996) )[2] < 10 )

    def testLine( self ):
        line = Line( (1,2), (3,4) )
        self.assertAlmostEqual( line.signedDistance( (2,3) ), 0.0, 10 )
        self.assertTrue( line.signedDistance( (0, 10) ) > 0 )

    def testLineFinished( self ):
        line = Line( (0,0), (10,0) )
        self.assertTrue( not line.finished( ( 5.0, 0.0 ) ) )
        self.assertTrue( line.finished( ( 11.0, 0.0 ) ) )
        self.assertTrue( line.finished( ( 11.0, 1.0 ) ) )

    def testLineSnap( self ):
        self.assertEqualCoords( Line( (0,0), (10,0) ).snap( (2,3) ), (2,0) )

    def testLineNearest( self ):
        self.assertEqual( Line( (1,2), (3,4) ).nearest( (7,7) ), ( (3,4), 5.0, 1 ) )
        self.assertEqual( Line( (1,2), (3,4) ).nearest( (0,0) ), ( (1,2), math.sqrt(5), 0 ) )
        pos, dist, type = Line( (1,2), (3,4) ).nearest( (2,3) )
        self.assertEqualCoords( pos, (2,3) )
        self.assertAlmostEqual( dist, 0.0 )
        self.assertEqual( type , -1 )


# vim: expandtab sw=4 ts=4
