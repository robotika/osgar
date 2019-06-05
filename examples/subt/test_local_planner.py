import unittest
import math

from local_planner import LocalPlanner


class SubTChallengeTest(unittest.TestCase):

    def test_usage(self):
        planner = LocalPlanner()
        scan = [0] * 270  # no obstacles
        planner.update(scan)
        self.assertEqual(planner.recommend(0), (1.0, 0.0))

        scan[135] = 1000
        scan[100] = 1000  # is the direction defined?!
        planner.update(scan)
        self.assertEqual(planner.recommend(0), 
                (0.4562465649202554, 1.2217304763960306))

        for i in range(135):
            scan[i] = 1000
        planner.update(scan)
        self.assertEqual(planner.recommend(0), 
                (0.4562465649202554, 1.2217304763960306))

# vim: expandtab sw=4 ts=4

