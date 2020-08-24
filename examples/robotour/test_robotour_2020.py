import unittest
import math
from examples.robotour.robotour_2020 import get_direction

class RobotourTest(unittest.TestCase):
    def test_get_direction(self):
        self.assertEqual(get_direction(1, 1), math.radians(45))
        self.assertEqual(get_direction(-0.5, -1), -2.0344439357957027) # approx. -126.6 deg
        self.assertEqual(get_direction(0, -1), math.radians(-90))
        self.assertEqual(get_direction(-1, 0), math.radians(180))


if __name__ == '__main__':
    unittest.main()
