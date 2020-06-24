import cv2
import numpy as np
import math

NO_MEASUREMENT = 0
MAX_OBSTACLE_DISTANCE = 20


def draw_scan(scan, max_obstacle_distance=None, scan_begin=-135, scan_end=135):
    if max_obstacle_distance is None:
        max_obstacle_distance = MAX_OBSTACLE_DISTANCE
    n = len(scan)
    scan = np.asarray(scan) / 1000

    angles = np.linspace(math.radians(scan_begin), math.radians(scan_end), n).reshape((1, -1))
    angles_cos = np.cos(angles)
    angles_sin = np.sin(angles)
    is_valid = scan != NO_MEASUREMENT
    valid_scan = scan[is_valid]
    is_valid = is_valid.reshape((1, -1))
    acoss = angles_cos[is_valid]
    asins = angles_sin[is_valid]
    x = acoss * valid_scan
    y = asins * valid_scan
    far_map = valid_scan > max_obstacle_distance

    height_px = 768
    width_px = 1024
    img = np.zeros((height_px, width_px, 3), dtype=np.uint8)

    scale = 50
    for ix, iy, is_far in zip(x, y, far_map):
        point = (width_px//2 - int(iy*scale), height_px//2 - int(ix*scale))
        color = (0, 255, 0) if not is_far else (120, 120, 120)
        cv2.circle(img, point, radius=3, color=color, thickness=-1)

    point = (width_px//2, height_px//2)
    point2 = (width_px//2, height_px//2-20)
    cv2.drawMarker(img, point, color=(0, 0, 255), markerType=cv2.MARKER_DIAMOND, thickness=3, markerSize=10)
    cv2.line(img, point, point2, thickness=3, color=(0, 0, 255))
    return img
