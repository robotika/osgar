import unittest
from unittest.mock import MagicMock
import math
import numpy as np
from examples.robotour.robotour_2020 import get_direction, Robotour, downsample_scan


class RobotourTest(unittest.TestCase):
    def test_get_direction(self):
        self.assertEqual(get_direction(1, 1), math.radians(45))
        self.assertEqual(get_direction(-0.5, -1), -2.0344439357957027) # approx. -126.6 deg
        self.assertEqual(get_direction(0, -1), math.radians(-90))
        self.assertEqual(get_direction(-1, 0), math.radians(180))

    def test_navigate(self):
        scan = np.ones(270)
        scan[135:] = 3
        bus = MagicMock()
        r = Robotour(bus=bus, config={"max_speed": 0.5})

        r.new_scan = scan*1000
        self.assertEqual(r.navigate(math.pi), math.pi/2)
        self.assertEqual(r.navigate(0), 0.6981317007977318)

        r.new_scan = scan[::-1] * 1000
        self.assertEqual(r.navigate(0), -0.6981317007977318)


if __name__ == '__main__':
    unittest.main()
