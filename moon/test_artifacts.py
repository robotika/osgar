import unittest
from unittest.mock import MagicMock
from pathlib import Path

import numpy as np
import cv2

from moon.artifacts import ArtifactDetector

curdir = Path(__file__).parent

class ArtifactTest(unittest.TestCase):

    def is_in_test(self, detected, valid):
        # note: classifier-based detection is not deterministic, ie it finds slightly different matches on different runs
        # we list all possible valid values and then test whether the detected values is within the set of valid ones
        self.assertEqual(len(detected), len(valid))
        for i in range(len(detected)):
            self.assertIn(detected[i], valid[i])

    def test_artifacts(self):

        bus = MagicMock()
        config = {}
        detector = ArtifactDetector(config, bus)
        for i in range(len(detector.detectors)):
            detector.detectors[i]["subsequent_detects_required"] = 0

        with open(str(curdir/'test_data/cube_homebase.jpg'), mode='rb') as img_h:
            img = img_h.read()

        # classifier detector returns one of two matches for this image, allow both
        self.is_in_test(detector.detect(img, img), [[('cubesat', 143, 24, 35, 40, 356)], [('homebase', 180, 185, 93, 93, 6264), ('homebase', 176, 140, 117, 114, 7158)], [('homebase', 202, 163, 71, 41, 1594)]])

        with open(str(curdir/'test_data/basemarker.jpg'), mode='rb') as img_h:
            img = img_h.read()
        self.is_in_test(detector.detect(img, img), [[('basemarker', 122, 174, 135, 104, 3848)], [('homebase', 1, 1, 349, 286, 8370)], [('rover', 314, 1, 84, 198, 9558)]])


# vim: expandtab sw=4 ts=4
