import unittest
from unittest.mock import MagicMock, patch, call

# https://stackoverflow.com/questions/8658043/how-to-mock-an-import
import sys
sys.modules['rospy'] = MagicMock()
sys.modules['sensor_msgs'] = MagicMock()
sys.modules['sensor_msgs.msg'] = MagicMock()
import filter_pointcloud


class FilterPointCloudTest(unittest.TestCase):

    def test_usage(self):
        fpc = filter_pointcloud.FilterPointCloud()
#        print(fpc.background)


if __name__ == "__main__":
    unittest.main()

# vim: expandtab sw=4 ts=4
