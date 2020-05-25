import unittest
import datetime

from subt.tools.validator import evaluate_poses
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


# vim: expandtab sw=4 ts=4
