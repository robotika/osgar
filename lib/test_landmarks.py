import unittest
from landmarks import *

class ConeLandmarkFinderTest(unittest.TestCase):

    def test_cone_detection(self):
        data = [0]*541
        cd = ConeLandmarkFinder()
        self.assertEqual(cd.find_cones(data), [])

        # single peak
        data[100] = 3000
        self.assertEqual(cd.find_cones(data), [(100, 3000)])

        # 5deg boundary peak
        data[99] = 3000
        self.assertEqual(cd.find_cones(data), [(99, 3000)])

if __name__ == "__main__":
    unittest.main() 

# vim: expandtab sw=4 ts=4 

