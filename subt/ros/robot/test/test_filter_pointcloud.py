import unittest
from unittest.mock import MagicMock, patch, call

import numpy as np

# https://stackoverflow.com/questions/8658043/how-to-mock-an-import
import sys
sys.modules['rospy'] = MagicMock()
sys.modules['sensor_msgs'] = MagicMock()
sys.modules['sensor_msgs.msg'] = MagicMock()
from subt.ros.robot.src import filter_pointcloud


class Msg:
    def __init__(self, data):
        self.height = 480
        self.width = 640
        self.point_step = 24
        self.row_step = 640 * 24
        self.header = None
        self.data = data


class FilterPointCloudTest(unittest.TestCase):

    def test_usage(self):
        # keep 1m array
        fpc = filter_pointcloud.FilterPointCloud()
        data = np.ones((480, 640, 6), dtype=np.float32)  # close x, y, z + 3 extra unused layers
        msg = Msg(data)
        new_data = fpc.filter_points(msg)
        arr = np.ones((480, 640, 3), dtype=np.float32)
        self.assertEqual(new_data, arr.tobytes())

    def test_propellers(self):
        fpc = filter_pointcloud.FilterPointCloud()
        data = np.ones((480, 640, 6), dtype=np.float32) * 0.1  # close x, y, z + 3 extra unused layers
        msg = Msg(data)
        new_data = fpc.filter_points(msg)
        data_minf = np.ones((480, 640, 3), dtype=np.float32) * float('-inf')
        self.assertEqual(new_data, data_minf.tobytes())

    def test_out_of_range(self):
        fpc = filter_pointcloud.FilterPointCloud()
        data = np.ones((480, 640, 6), dtype=np.float32) * float('inf')  # close x, y, z + 3 extra unused layers
        msg = Msg(data)
        new_data = fpc.filter_points(msg)
        self.assertEqual(np.frombuffer(new_data, dtype=np.float32).max(), 11.0)

if __name__ == "__main__":
    unittest.main()

# vim: expandtab sw=4 ts=4
