"""
    Analyze 2D lidar scan points
"""
import math

import cv2
import numpy as np

from osgar.lib.line import distance


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
    hull_pts = [p[0] for p in hull] + [hull[0][0],]
    dist_arr = [distance(a, b) for a, b in zip(hull_pts[:-1], hull_pts[1:])]
    #print('%.2f %.2f %.2f %.2f' % tuple(sorted(dist_arr, reverse=True)[:4]))
    first, second = sorted(dist_arr, reverse=True)[:2]
    i, j = dist_arr.index(first), dist_arr.index(second)  # TODO argmax
    A = ((hull_pts[i][0] + hull_pts[i+1][0])/2, (hull_pts[i][1] + hull_pts[i+1][1])/2)
    B = ((hull_pts[j][0] + hull_pts[j+1][0])/2, (hull_pts[j][1] + hull_pts[j+1][1])/2)
    if debug_poly is not None:
        #debug_poly.append(hull_pts)
        debug_poly.append([A, B])
    return (A, B)


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


def equal_scans(scan1, scan2, tollerance=10):
    """
    Compare two scans for "almost identity". Expected inputs are raw scans (in millimeters).
    """
    assert len(scan1) == len(scan2), (len(scan1), len(scan2))
    arr1 = np.array(scan1)
    arr2 = np.array(scan2)
    diff = arr1 - arr2
    return np.abs(diff).max() <= tollerance

# vim: expandtab sw=4 ts=4
