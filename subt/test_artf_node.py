import unittest

import numpy as np

from subt.artf_node import result2report, check_results, result2list, cv_result2list


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

    def test_result2list(self):
        result = [('backpack', [(20, 4, 0.9549203), (16, 4, 0.96661633), (24, 8, 0.9785418)])]
        self.assertEqual(result2list(result),
                         [['backpack', [[20, 4, 0.9549203], [16, 4, 0.96661633], [24, 8, 0.9785418]]]])
        result = [('backpack', [(624, 20, 0.97964704), (628, 20, 0.98307204), (624, 24, 0.97980016), (628, 24, 0.96485513)])]
        self.assertEqual(result2list(result),
                         [['backpack', [[624, 20, 0.97964704], [628, 20, 0.98307204], [624, 24, 0.97980016], [628, 24, 0.96485513]]]])

    def test_cv_result2list(self):
        result = [('helmet', 0.6372756, np.array([614, 224, 637, 246], dtype=np.int32))]
        self.assertEqual(cv_result2list(result),
                         [['helmet', 0.6372756, [614, 224, 637, 246]]])
        self.assertEqual(cv_result2list([]), [])

# vim: expandtab sw=4 ts=4
