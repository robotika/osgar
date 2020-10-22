import unittest

import numpy as np

from subt.artf_node import result2report, check_results


DRONE_FX = 554.25469

class ArtifactDetectorDNNTest(unittest.TestCase):

    def test_result2report(self):
        result = [('backpack', [(60, 180, 0.9785775), (72, 180, 0.9795098), (60, 184, 0.97716093), (72, 184, 0.9782014)])]
        row = [5000]*640
        depth = np.array([row]*360, dtype=np.uint16)
        self.assertEqual(result2report(result, depth, fx=DRONE_FX), ['TYPE_BACKPACK', [5000, 2291, -18]])

        row = list(range(640))
        depth = np.array([row]*360, dtype=np.uint16)
        self.assertEqual(result2report(result, depth, fx=DRONE_FX), ['TYPE_BACKPACK', [66, 30, 0]])

        result2 = [('rope', [(400, 180, 0.9785775)])]
        self.assertEqual(result2report(result2, depth, fx=DRONE_FX), ['TYPE_ROPE', [400, -57, 0]])

    def test_out_of_range(self):
        result = [('backpack', [(60, 180, 0.9785775), (72, 180, 0.9795098), (60, 184, 0.97716093), (72, 184, 0.9782014)])]
        row = [0xFFFF]*640
        depth = np.array([row]*360, dtype=np.uint16)
        self.assertEqual(result2report(result, depth, fx=DRONE_FX), None)

    def test_check_reults(self):
        result = [('backpack', [(60, 180, 0.9785775), (72, 180, 0.9795098)]),
                  ('backpack', [(260, 10, 0.9785775), (261, 10, 0.9795098)]),  # out of bbox, false detection
                  ('rope', [(400, 100, 0.9785775), (401, 100, 0.9795098)])
                  ]
        result_cv = [['backpack', 0.99990773, [50, 150, 200, 250]],
                     ['rope', 0.99650773, [350, 90, 500, 250]]
                     ]
        expected_result = [('backpack', [(60, 180, 0.9785775), (72, 180, 0.9795098)]),
                           ('rope', [(400, 100, 0.9785775), (401, 100, 0.9795098)])
                          ]
        checked_result = check_results(result, result_cv)
        self.assertEqual(checked_result, expected_result)

    def test_avoid_double_detection(self):
        result = [('backpack', [(100, 200, 0.9785775), (101, 200, 0.9795098)])]
        result_cv = [['backpack', 0.99990773, [50, 150, 200, 250]], ['backpack', 0.99990773, [60, 150, 210, 250]]]
        checked_result = check_results(result, result_cv)
        self.assertEqual(checked_result, result)

    def test_merge_two_results(self):
        result = [('backpack', [(100, 200, 0.9785775), (101, 200, 0.9795098)]),
                  ('backpack', [(102, 200, 0.9785775), (103, 200, 0.9795098)])
                  ]
        result_cv = [['backpack', 0.99990773, [50, 150, 200, 250]]]
        expected_result = [('backpack', [(100, 200, 0.9785775), (101, 200, 0.9795098),
                                         (102, 200, 0.9785775), (103, 200, 0.9795098)])
                           ]
        checked_result = check_results(result, result_cv)
        self.assertEqual(checked_result, expected_result)

    def test_report_without_depth(self):
        result = [('backpack', [(100, 200, 0.9785775), (101, 200, 0.9795098)]),
                  ('backpack', [(102, 200, 0.9785775), (103, 200, 0.9795098)])
                  ]
        self.assertIsNone(result2report(result, depth=None, fx=100))

# vim: expandtab sw=4 ts=4
