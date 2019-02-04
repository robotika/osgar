"""
    Analyze 2D lidar scan points
"""
import math

import cv2
import numpy as np


def rect(scan, debug_poly=None):
    """
    Dummy function to test debug polygons of lidar viewer
    """
    if debug_poly is not None:
        debug_poly.append([(0, 0), (1, 0), (1, 1), (0, 1), (0, 0)])


def convex_hull(scan, debug_poly=None):
    """
    Test visualization of convex hull
    """
    heading = 0.0  # TODO
    pts = []
    for i, i_dist in enumerate(scan):
        if i_dist == 0 or i_dist >= 10000:
            continue
        angle = math.radians(270 * (i / len(scan)) - 135) + heading
        dist = i_dist/1000.0
        x, y = dist * math.cos(angle), dist * math.sin(angle)
        pts.append((x,y))

    hull = cv2.convexHull(np.array(pts, dtype=np.float32))
    if debug_poly is not None:
        debug_poly.append([p[0] for p in hull])
        debug_poly[-1].append(hull[0][0])  # close polygon


def dual_convex_hull(scan, debug_poly=None):
    """
    Compute convex hull for left and right side of the scan
    (usefull if already in tunnel)
    """
    heading = 0.0  # TODO
    pts_left, pts_right = [], []
    for i, i_dist in enumerate(scan):
        if i_dist == 0 or i_dist >= 10000:
            continue
        angle = math.radians(270 * (i / len(scan)) - 135) + heading
        dist = i_dist/1000.0
        x, y = dist * math.cos(angle), dist * math.sin(angle)
        if angle >= 0:  # beware of heading modification, TODO autodetect
            pts_left.append((x, y))
        else:
            pts_right.append((x, y))

    hull_left = cv2.convexHull(np.array(pts_left, dtype=np.float32))
    hull_right = cv2.convexHull(np.array(pts_right, dtype=np.float32))
    if debug_poly is not None:
        debug_poly.append([p[0] for p in hull_left])
        debug_poly[-1].append(hull_left[0][0])  # close polygon
        debug_poly.append([p[0] for p in hull_right])
        debug_poly[-1].append(hull_right[0][0])  # close polygon

# vim: expandtab sw=4 ts=4
