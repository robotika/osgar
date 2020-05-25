import unittest
import datetime

from subt.tools.validator import evaluate_poses, ign2arr, osgar2arr
from subt.ign_pb2 import Vector3d


class ValidatorTest(unittest.TestCase):

    def test_evaluate_poses(self):
        self.assertIsNone(evaluate_poses([], []))

        vec = Vector3d()
        vec.x = 1
        vec.y = 2
        vec.z = 0.5
        gt = [(datetime.timedelta(0, 1, 548000), {'A60F300L': vec})]
        poses = [(datetime.timedelta(0, 55), [[0.0, 0.0, 0.0], [1, 0, 0, 0]])]
        self.assertIsNone(evaluate_poses(poses, gt))

    def test_ign2arr(self):
        vec = Vector3d()
        vec.x = 1
        vec.y = 2
        vec.z = 0.5
        gt = [(datetime.timedelta(0, 1, 548000), {'A60F300L': vec})]
        arr = ign2arr(gt, robot_name='A60F300L')
        self.assertEqual(arr, [(1.548, 1, 2, 0.5)])

        arr2 = ign2arr(gt, robot_name='B90F300R')
        self.assertEqual(arr2, [])

    def test_osgar2arr(self):
        poses = [(datetime.timedelta(0, 55), [[0.0, 0.0, 42.0], [1, 0, 0, 0]])]
        arr = osgar2arr(poses)
        self.assertEqual(arr, [(55.0, 0.0, 0.0, 42.0)])

# vim: expandtab sw=4 ts=4
