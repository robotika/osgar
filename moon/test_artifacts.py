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
                if abs(d[5] - t[i][5]) > 400:
                    continue
                if len(d) != len(t[i]):
                    continue
                if len(d) == 7 and abs(d[6] - t[i][6]) > 1: #distance difference more than 1m
                    continue
                return True
            return False

        self.assertEqual(len(detected), len(valid))
        for i in range(len(valid)):
            self.assertGreaterEqual(len(detected[i]), 6) # color matches have 7 fields (+distance), classifier matches 6
            if not comp_tuple(detected[i], valid[i]):
                return False
        return True


    def test_artifacts(self):

        bus = MagicMock()
        config = {"estimate_distance": True, "artefacts": ["rover","cubesat","homebase", "basemarker"]}
        detector = ArtifactDetector(config, bus)
        for i in range(len(detector.detectors)):
            detector.detectors[i]["subsequent_detects_required"] = 0

        with open(str(curdir/'test_data/cube_homebase.jpg'), mode='rb') as img_h:
            img = img_h.read()

        # TODO: artifact now also calculates distance from stereo images, use stereo images for unittest
        # classifier detector returns one of two matches for this image, allow both
        self.assertTrue(self.is_in_test(detector.detect(img, img), [[('cubesat', 143, 24, 35, 40, 356)], [('homebase', 180, 185, 93, 93, 6264), ('homebase', 176, 140, 117, 114, 7158)], [('homebase', 202, 163, 71, 86, 2062, 1.6899802684783936)]]))

        with open(str(curdir/'test_data/basemarker.jpg'), mode='rb') as img_h:
            img = img_h.read()
        self.assertTrue(self.is_in_test(detector.detect(img, img), [[('basemarker', 122, 240, 135, 36, 2068, 6.632953643798828)], [('homebase', 0, 0, 429, 287, 14012, 1.5621232986450195)], [('rover', 347, 0, 81, 284, 1107, 1.1481255292892456)]]))


# vim: expandtab sw=4 ts=4
