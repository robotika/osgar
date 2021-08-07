import math
import unittest
from unittest.mock import MagicMock

from subt.crash_report import CrashReport


class CrashReportTest(unittest.TestCase):

    def test_usage(self):
        bus = MagicMock()
        black_box = CrashReport(bus=bus, config={})

        data = b'data1'
        black_box.on_rgbd(data)
        bus.pubilsh.assert_not_called()
        black_box.on_acc(([-60, 70, 9805]))
        bus.publish.assert_not_called()

        data = b'data2'
        black_box.on_rgbd(data)
        bus.publish.assert_not_called()

        # example crash of drones in ver115tf2fp1r2-d53b93ce-64e7-43fc-9838-3ee59b6b77b6, A-drone
        # 0:13:22.940140 9 [-144200, 3600, 3245]
        # 0:13:23.024618 9 [-220, 125, 9880]
        # 0:13:23.031486 9 [-55, 90, 55]
        # 0:13:23.035212 9 [-10, 95, 85]
        # 0:13:23.041146 9 [-20, 130, 55]
        # 0:13:23.188065 9 [5, 90, 65]
        # 0:13:23.190759 9 [5, 80, 95]
        # 0:13:23.197192 9 [30, 95, 65]
        # 0:13:23.203757 9 [-10, 65, 25]
        # 0:13:23.275979 9 [-2250, 3700, 630]
        # 0:13:23.513236 9 [-288580, -65640, -56455]

        black_box.on_acc(([-144200, 3600, 3245]))
        bus.publish.assert_called_with('crash_rgbd', b'data2')

        bus.reset_mock()
        black_box.on_acc(([-288580, -65640, -56455]))  # now new data were received in meantime
        bus.publish.assert_not_called()

# vim: expandtab sw=4 ts=4
