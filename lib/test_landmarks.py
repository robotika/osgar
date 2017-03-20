import unittest
from landmarks import *

class LandmarksTest(unittest.TestCase):

    def test_cone_detection(self):
        data = [0]*541
        cd = ConeLandmarks(data)
        self.assertEqual(cd.find_cones(), [])


if __name__ == "__main__":
    unittest.main() 
  
# vim: expandtab sw=4 ts=4 

