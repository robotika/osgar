import unittest
import math
from unittest.mock import MagicMock
from datetime import timedelta

from osgar.explore import tangent_circle, follow_wall_angle


class ExploreTest(unittest.TestCase):

    def test_tangent_circle(self):
        angle = tangent_circle(dist=2.0, radius=1.0)
#        self.assertAlmostEqual(angle, math.radians(30))
        self.assertIsNone(angle)  # too far

        angle = tangent_circle(dist=2.0, radius=3.0)
#        self.assertIsNone(angle)  # no tangent
        self.assertAlmostEqual(angle, math.radians(100))

        angle = tangent_circle(dist=math.sqrt(2), radius=1.0)
        self.assertAlmostEqual(angle, math.radians(45))

    def test_follow_wall_angle(self):
        # nothing close -> no command
        scan = [0] * 271  # 1deg resolution
        cmd = follow_wall_angle(scan, radius=1.0, right_wall=True)
        self.assertAlmostEqual(cmd, math.radians(-20))

        # parallel wall at 1m on the right
        scan = []
        for i in range(-135, 135+1):
            # ax + by + c = 0
            #  y = -1
            # ray: (t*cos(angle), t*sin(angle))
            #  t*sin(angle) = -1
            #  t = -1/sin(angle)
            if i < 0:
                dist_mm = -round(1000/math.sin(math.radians(i)))
            else:
                dist_mm = 0  # no reflection
            scan.append(dist_mm)
        cmd = follow_wall_angle(scan, radius=1.0, right_wall=True)
        self.assertIsNotNone(cmd)
        # angle should be 0.0, but due to resolution it corresponds to 1deg
        self.assertLess(abs(math.degrees(cmd)), 1.5)

        # move towards wall
        cmd = follow_wall_angle(scan, radius=0.7, right_wall=True)
        self.assertIsNotNone(cmd)
        self.assertLess(math.degrees(cmd), -45)

        # move from wall
        cmd = follow_wall_angle(scan, radius=1.3, right_wall=True)
        self.assertIsNotNone(cmd)
        self.assertGreater(math.degrees(cmd), 8.0)

# vim: expandtab sw=4 ts=4
