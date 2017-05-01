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

    def test_match_pairs(self):
        finder = ConeLandmarkFinder()
        self.assertEqual(finder.match_pairs([], []), [])

        res = finder.match_pairs([(312, 11509)], [(264, 10425), (312, 11477)])
        self.assertEqual(res, [((312, 11509), (312, 11477))])

        old = [            (199, 5576), (488, 9282), (510, 19111)]
        new = [(94, 5386), (195, 5562), (486, 9295), (508, 19193)]
        res = finder.match_pairs(old, new)
        self.assertEqual(res, [((199, 5576), (195, 5562)),
                               ((488, 9282), (486, 9295)),
                               ((510, 19111), (508, 19193))])

        self.assertEqual(finder.match_pairs([(100,1000)], [(100,1300)]), [])

    def test_pair_distance(self):
        finder = ConeLandmarkFinder()
        self.assertAlmostEqual(finder.pair_distance((100, 1000), (100, 1000)), 0.0)

        self.assertAlmostEqual(finder.pair_distance((100, 1000), (100, 1100)), 0.1)

if __name__ == "__main__":
    unittest.main() 

# vim: expandtab sw=4 ts=4 

