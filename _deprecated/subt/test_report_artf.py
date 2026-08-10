import unittest
from unittest.mock import patch
import math

from subt.report_artf import triple, score


class ReportArtfTest(unittest.TestCase):

    def test_triple(self):
        arr = triple(0, 0, 0)
        self.assertEqual(len(arr), 3)
        self.assertEqual(arr[0], (4, 0, 0))
        self.assertAlmostEqual(arr[1][0], -2)
        self.assertAlmostEqual(arr[1][1], math.sqrt(12))
        self.assertAlmostEqual(arr[2][0], -2)
        self.assertAlmostEqual(arr[2][1], -math.sqrt(12))

        arr = triple(10, 0, 0)
        self.assertEqual(len(arr), 3)
        self.assertEqual(arr[0], (14, 0, 0))


    def test_score(self):
        with patch('subt.report_artf.get_status') as p, \
             patch('subt.report_artf.report_artf') as p2, \
             patch('time.sleep'):
            p.side_effect = [
                b'{"score":0,"remaining_reports":38,"current_team":"robotika","run_clock":983.1}',
                b'{"score":1,"remaining_reports":37,"current_team":"robotika","run_clock":987.4}',
            ]
            self.assertTrue(score('Survivor', 0, 0, 0))

            p.side_effect = [
                b'{"score":0,"remaining_reports":39,"current_team":"robotika","run_clock":966.5}',
                b'{"score":0,"remaining_reports":38,"current_team":"robotika","run_clock":970.8}',
            ]
            self.assertFalse(score('Backpack', 1, 2, 3))

# vim: expandtab sw=4 ts=4

