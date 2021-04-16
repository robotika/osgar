import unittest
from unittest.mock import MagicMock, patch, call

import numpy as np

# https://stackoverflow.com/questions/8658043/how-to-mock-an-import
import sys
sys.modules['rospy'] = MagicMock()
sys.modules['sensor_msgs'] = MagicMock()
sys.modules['sensor_msgs.msg'] = MagicMock()
import filter_pointcloud


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
        fpc.points_publisher = MagicMock()
        data = np.ones((480, 640, 6), dtype=np.float32)  # close x, y, z + 3 extra unused layers
        msg = Msg(data)
        fpc.points_callback(msg)
        fpc.points_publisher.publish.assert_called()
        arr = np.ones((480, 640, 3), dtype=np.float32)
        self.assertEqual(fpc.debug_data, arr.tobytes())

    def test_propellers(self):
        fpc = filter_pointcloud.FilterPointCloud()
        fpc.points_publisher = MagicMock()
        data = np.ones((480, 640, 6), dtype=np.float32) * 0.1  # close x, y, z + 3 extra unused layers
        msg = Msg(data)
        fpc.points_callback(msg)
        data_minf = np.ones((480, 640, 3), dtype=np.float32) * float('-inf')
        self.assertEqual(fpc.debug_data, data_minf.tobytes())

    def test_out_of_range(self):
        fpc = filter_pointcloud.FilterPointCloud()
        data = np.ones((480, 640, 6), dtype=np.float32) * float('inf')  # close x, y, z + 3 extra unused layers
        msg = Msg(data)
        fpc.points_callback(msg)
        self.assertEqual(np.frombuffer(fpc.debug_data, dtype=np.float32).max(), 11.0)

if __name__ == "__main__":
    unittest.main()

# vim: expandtab sw=4 ts=4
