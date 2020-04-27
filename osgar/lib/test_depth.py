import unittest
import math

import numpy as np

from osgar.lib import depth

from osgar.lib.depth import depth2dist


class DepthTest(unittest.TestCase):

    def test_noop(self):
        dimage = np.zeros((depth.CAMH, depth.CAMW), dtype="int32")
        scan = depth.depth2dist(dimage)
        self.assertTrue(np.all(scan == 0))

    def test_1m(self):
        dimage = np.ones((depth.CAMH, depth.CAMW), dtype="int32") * 1000
        scan = depth.depth2dist(dimage)
        middle = scan[len(scan)//2]
        right = scan[0]
        left = scan[-1]

        import math
        expected_x = 1 + depth.CAMX
        expected_y = (depth.CAMW / 2) / depth.FX
        expected_dist = math.hypot(expected_x, expected_y)

        self.assertAlmostEqual(middle/1000.0, expected_x)
        self.assertAlmostEqual(left/1000.0, expected_dist, delta=0.01)
        self.assertAlmostEqual(right/1000.0, expected_dist, delta=0.01)
        self.assertTrue(np.all(scan >= expected_x*1000))
        self.assertTrue(np.all(scan <= expected_dist*1000))

    def test_ground(self):
        dimage = np.ones((depth.CAMH, depth.CAMW), dtype="int32")*depth.MAXX*1000

        for py in range( -depth.CAMH // 2 + 1, 0):
            for px in range( -depth.CAMW // 2, depth.CAMW // 2 ):
                x = (depth.CAMZ * depth.FX) / py
                y = (px / depth.FX) * x
                d1 = math.hypot(x, y)
                d = math.hypot(d1, depth.CAMZ)
                dimage[depth.CAMH // 2 - py, depth.CAMW // 2 + px] = d*1000

        #_show_depth(dimage)
        scan = depth.depth2dist(dimage)
        self.assertTrue(np.all(scan == 0))
        #_show_scan(scan)
        #wait()

    def test_cliff(self):
        dimage = np.ones((depth.CAMH, depth.CAMW), dtype="int32")*depth.MAXX*1000

        for py in range( -depth.CAMH // 2 + 1, 0):
            for px in range( -depth.CAMW // 2, depth.CAMW // 2 ):
                x = (depth.CAMZ * depth.FX) / py
                y = (px / depth.FX) * x
                if y > 0.5:
                    d = depth.MAXX
                else:
                    d1 = math.hypot(x, y)
                    d = math.hypot(d1, depth.CAMZ)
                dimage[depth.CAMH // 2 - py, depth.CAMW // 2 + px] = d*1000

        #_show_depth(dimage)
        scan = depth.depth2dist(dimage)
        #_show_scan(scan)
        #wait()
        self.assertTrue(np.any(scan != 0))


def _show_scan(scan):
    from osgar.lib import drawscan
    import cv2
    import math
    expected_x = 1 + depth.CAMX
    expected_y = (depth.CAMW / 2) / depth.FX
    angle = math.degrees(math.atan2(expected_y, expected_x))
    img = drawscan.draw_scan(scan, scan_begin=-angle, scan_end=angle)
    cv2.imshow("depth2dist", img)


def _show_depth(data):
    import cv2
    img = np.array(np.minimum(255 * 40, data) / 40, dtype=np.uint8)
    im_color = cv2.applyColorMap(img, cv2.COLORMAP_JET)
    cv2.imshow("depth", im_color)


def wait():
    import cv2
    while True:
        key = cv2.waitKey()
        if key == 27:
            return

# vim: expandtab sw=4 ts=4
