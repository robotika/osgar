import unittest
import math

from .line import Line, pointAtPolyLineDist


class LineTest(unittest.TestCase):

    def test_signed_distance(self):
        line = Line((0, 0), (1, 1))
        self.assertAlmostEqual(line.signedDistance((0.5, 0.5)), 0.0)
        self.assertAlmostEqual(line.signedDistance((0, 1)), math.sqrt(1/2))
        self.assertAlmostEqual(line.signedDistance((1, 0)), -math.sqrt(1/2))

        # it is a infinite line
        self.assertAlmostEqual(line.signedDistance((2, 2)), 0.0)

    def test_distance_to_finish(self):
        line = Line((0, 0), (1, 1))
        self.assertAlmostEqual(line.distanceToFinishLine((0, 0)), math.sqrt(2))
        self.assertAlmostEqual(line.distanceToFinishLine((-1, 1)), math.sqrt(2))
        self.assertAlmostEqual(line.distanceToFinishLine((2, 2)), -math.sqrt(2))

        self.assertTrue(line.finished((2, 2)))

    def test_snap(self):
        line = Line((0, 0), (1, 1))
        p = line.snap((1, 0))
        self.assertAlmostEqual(p[0], 0.5)
        self.assertAlmostEqual(p[1], 0.5)

        p = line.snap((2, 2))
        self.assertAlmostEqual(p[0], 2)
        self.assertAlmostEqual(p[1], 2)

    def test_nearest(self):
        line = Line((0, 0), (1, 1))
        p, abs_dist, index = line.nearest((2, 2))
        self.assertAlmostEqual(p[0], 1)
        self.assertAlmostEqual(p[1], 1)
        self.assertAlmostEqual(abs_dist, math.sqrt(2))
        self.assertEqual(index, 1)

        p, abs_dist, index = line.nearest((1, 0))
        self.assertAlmostEqual(p[0], 0.5)
        self.assertAlmostEqual(p[1], 0.5)
        self.assertAlmostEqual(abs_dist, abs(-math.sqrt(1/2)))
        self.assertEqual(index, -1)

    def test_point_at_dist(self):
        polyline = [(0, 0), (1, 0), (1, 2)]
        p = pointAtPolyLineDist(polyline, 2.0)
        self.assertAlmostEqual(p[0], 1)
        self.assertAlmostEqual(p[1], 1)

# vim: expandtab sw=4 ts=4

