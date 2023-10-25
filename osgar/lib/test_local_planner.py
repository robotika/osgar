import unittest
import math

from subt.local_planner import LocalPlanner, LocalPlannerRef, LocalPlannerOpt, LocalPlannerNumpy


class SubTChallengeTest(unittest.TestCase):

    def test_usage(self):
        planner = LocalPlanner()
        scan = [0] * 270  # no obstacles
        planner.update(scan)
        self.assertEqual(planner.recommend(0), (1.0, 0.0))

        scan[135] = 1000
        scan[100] = 1000  # the direction from right to left
        planner.update(scan)
        good, direction = planner.recommend(0)
        self.assertAlmostEqual(good, 0.47525253623471064)
        self.assertAlmostEqual(direction, 1.3089969389957472)

        # extra obstacles should not influence the decision
        for i in range(135):
            scan[i] = 1000
        planner.update(scan)
        good, direction = planner.recommend(0)
        self.assertAlmostEqual(good, 0.47525253623471064)
        self.assertAlmostEqual(direction, 1.3089969389957472)

    def test_corner(self):
        planner = LocalPlannerOpt()
        # obstacle defined as line "x + y - 1 = 0", corner defined via abs()
        # ray defined as t*cos(angle), t*sin(angle)
        # solution for t = 1/(cos(angle) + sin(angle))
        angles = [abs(math.radians(i-135)) for i in range(271)]
        scan = [int(1400 / (math.cos(a) + math.sin(a))) for a in angles]
        planner.update(scan)
        self.assertEqual(planner.recommend(0), (0.18451952399298924, 2.0420352248333655))

        # now test also old planner
        planner_ref = LocalPlannerRef()
        planner_ref.update(scan)
        self.assertEqual(planner.recommend(0), planner_ref.recommend(0))

        # but why numpy is correct?
        planner = LocalPlannerNumpy()
        planner.update(scan)
        self.assertEqual(planner.recommend(0), (0.18451952399298924, -2.0420352248333655))

# vim: expandtab sw=4 ts=4

