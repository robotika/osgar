"""
    Osgar driver for Luxonis OAK cameras.
    https://www.luxonis.com/
"""
import depthai

if depthai.__version__ < '3.0':
    # depthai v2
    from osgar.drivers.oak_camera_v2 import *
else:
    # depthai v3
    from osgar.drivers.oak_camera_v3 import *
