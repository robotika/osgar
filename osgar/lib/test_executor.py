#!/usr/bin/python

from mock import MagicMock
import math
import unittest

from driver import Driver

class DriverTest(unittest.TestCase):

    def test_followLineG( self ):
        robot = MagicMock()
        robot.localisation.pose.return_value = (0, 0.3, 0)
        driver = Driver(robot, maxSpeed = 0.5)
        offsetDistance = 0.03
        gen = driver.followPolyLineG(pts = [(0, 0), (1.0, 0)],
                                     offsetDistance=offsetDistance)
        speed, angularSpeed = gen.next()
        self.assertAlmostEqual(angularSpeed, -4 * math.radians(20))
        self.assertAlmostEqual(speed, 0.444444444444)

        # now for closer distance reduce the turning angle
        robot.localisation.pose.return_value = (0, 1.5 * offsetDistance, 0)
        speed, angularSpeed = gen.next()
        self.assertAlmostEqual(angularSpeed, -4 * math.radians(10))
        self.assertAlmostEqual(speed, 0.4722222222222222)




if __name__ == "__main__":
  unittest.main()

#-------------------------------------------------------------------
# vim: expandtab sw=4 ts=4
