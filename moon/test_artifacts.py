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
        # also, allow difference of 5 pixels in coordinates and 200 pixels in overall matching pixel count
        def comp_one(d, t):
            for j in range(1,5):
                if abs(d[j] - t[j]) > 5:
                    return False
            return True

        def comp_tuple(d, t):
            for i in range(len(t)):
                if d[0] != t[i][0]:
                    continue
                if not comp_one(d, t[i]):
                    continue
                if abs(d[5] - t[i][5]) > 200:
                    continue
                return True
            return False

        self.assertEqual(len(detected), len(valid))
        for i in range(len(valid)):
            self.assertEqual(len(detected[i]), 6)
            if not comp_tuple(detected[i], valid[i]):
                return False
        return True


    def test_artifacts(self):

        bus = MagicMock()
        config = {}
        detector = ArtifactDetector(config, bus)
        for i in range(len(detector.detectors)):
            detector.detectors[i]["subsequent_detects_required"] = 0

        with open(str(curdir/'test_data/cube_homebase.jpg'), mode='rb') as img_h:
            img = img_h.read()

        # classifier detector returns one of two matches for this image, allow both
        self.assertTrue(self.is_in_test(detector.detect(img, img), [[('cubesat', 143, 24, 35, 40, 356)], [('homebase', 180, 185, 93, 93, 6264), ('homebase', 176, 140, 117, 114, 7158)], [('homebase', 202, 163, 71, 41, 1594)]]))

        with open(str(curdir/'test_data/basemarker.jpg'), mode='rb') as img_h:
            img = img_h.read()
        self.assertTrue(self.is_in_test(detector.detect(img, img), [[('basemarker', 122, 174, 135, 104, 3848)], [('homebase', 1, 1, 349, 286, 8370)], [('rover', 314, 1, 84, 198, 9558)]]))


# vim: expandtab sw=4 ts=4
