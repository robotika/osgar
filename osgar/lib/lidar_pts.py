"""
    Analyze 2D lidar scan points
"""

def rect(scan, debug_poly=None):
    """
    Dummy function to test debug polygons of lidar viewer
    """
    if debug_poly is not None:
        debug_poly.append([(0, 0), (1, 0), (1, 1), (0, 1), (0, 0)])

# vim: expandtab sw=4 ts=4
