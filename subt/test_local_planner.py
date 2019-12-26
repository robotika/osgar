import unittest
import math

from subt.local_planner import LocalPlanner


class SubTChallengeTest(unittest.TestCase):

    def test_usage(self):
        planner = LocalPlanner()
        scan = [0] * 270  # no obstacles
        planner.update(scan)
        self.assertEqual(planner.recommend(0), (1.0, 0.0))

        scan[135] = 1000
        scan[100] = 1000  # the direction from right to left
        planner.update(scan)
        self.assertEqual(planner.recommend(0), 
                (0.47525253623471064, 1.3089969389957472))

        # extra obstacles should not influence the decision
        for i in range(135):
            scan[i] = 1000
        planner.update(scan)
        self.assertEqual(planner.recommend(0), 
                (0.47525253623471064, 1.3089969389957472))

# vim: expandtab sw=4 ts=4

