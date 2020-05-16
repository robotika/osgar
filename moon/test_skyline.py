import os
import unittest
from unittest.mock import MagicMock

import cv2
import numpy as np

from moon.skyline import skyline, draw_skyline, find_peaks


class SkylineTest(unittest.TestCase):
    def test_usage(self):
        img = cv2.imread(os.path.join(os.path.dirname(__file__), 'test_data', 'skyline.jpg'))
        self.assertEqual(img.shape, (480, 640, 3))

        s = skyline(img)
        self.assertEqual(len(s), 640)
        img2 = draw_skyline(img, s)
#        cv2.imshow('debug', img2)
#        cv2.waitKey(0)
#        cv2.imwrite('debug.jpg', img2)

    def test_horizon(self):
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        s = skyline(img)
        self.assertEqual(len(s), 640)
        self.assertEqual(s[42], 0)

        img[100:,:,:] = 120
        s = skyline(img)
        self.assertEqual(s[42], 100)

    def test_find_peaks(self):
        s = np.zeros(640, dtype=np.uint16)
        self.assertEqual(find_peaks(s), [])

        s[:] = 100
        s[13] = 40
        self.assertEqual(find_peaks(s), [(13, 40)])


# vim: expandtab sw=4 ts=4

