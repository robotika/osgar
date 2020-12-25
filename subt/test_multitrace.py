
import unittest
from unittest.mock import MagicMock, patch, call

from subt.multitrace import MultiTraceManager, get_intervals


class MultiTraceManagerTest(unittest.TestCase):

    def test_usage(self):
        bus = MagicMock()
        mtm = MultiTraceManager(bus=bus, config={})
        mtm.on_robot_xyz(['A150L', [505, [6.127597694909277, 2.640278491024048, 1.9188244676330934]]])
        mtm.on_robot_xyz(['A150L', [506, [5.07939188538461, 2.0835104023227147, 1.9305388336683675]]])
        bus.publish.assert_called()
        self.assertEqual(bus.method_calls[-1],
                         call.publish('trace_info', {'A150L': [[505, 506]]}))

        bus.reset_mock()
        mtm.on_trace_info({'B100R': [[1, 10]]})
        bus.publish.assert_called()
        self.assertEqual(bus.method_calls[-1],
                         call.publish('robot_trace', {'A150L': [[505, [6.127597694909277, 2.640278491024048, 1.9188244676330934]],
                                                                [506, [5.07939188538461, 2.0835104023227147, 1.9305388336683675]]]}))

        bus.reset_mock()
        mtm.on_trace_info({'A150L': [[505, 506]]})
        bus.publish.assert_not_called()

    def test_on_robot_trace(self):
        bus = MagicMock()
        mtm = MultiTraceManager(bus=bus, config={})
        mtm.on_robot_trace({'A150L': [[505, [6.127597694909277, 2.640278491024048, 1.9188244676330934]],
                                      [506, [5.07939188538461, 2.0835104023227147, 1.9305388336683675]]]})
        self.assertIn('A150L', mtm.traces)
        mtm.on_robot_trace({'A150L': [[500, [10.098783014125825, 2.4058896366987415, 2.0341068401239615]]]})
        self.assertEqual(len(mtm.traces['A150L']), 3)
        mtm.on_robot_trace({'A150L': [[500, [10.098783014125825, 2.4058896366987415, 2.0341068401239615]]]})
        self.assertEqual(len(mtm.traces['A150L']), 3)

        bus.reset_mock()
        mtm.publish_trace_info()
        bus.publish.assert_called()
        self.assertEqual(bus.method_calls[-1],
                         call.publish('trace_info', {'A150L': [[500, 500], [505, 506]]}))

    def test_get_intervals(self):
        self.assertEqual(get_intervals([1, 2, 3, 4]), [[1, 4]])
        self.assertEqual(get_intervals([]), [])
        self.assertEqual(get_intervals([1, 3, 5]), [[1, 1], [3, 3], [5, 5]])

    def test_partial_update(self):
        bus = MagicMock()
        mtm = MultiTraceManager(bus=bus, config={})
        for i in range(10):
            mtm.on_robot_xyz(['A10L', [i, [2*i, 0, 0]]])

        bus.reset_mock()
        mtm.on_trace_info({'A10L': [[1, 8]]})
        bus.publish.assert_called()
        self.assertEqual(bus.method_calls[-1],
                         call.publish('robot_trace', {'A10L': [[0, [0, 0, 0]], [9, [18, 0, 0]]]}))

    def test_too_large_update(self):
        bus = MagicMock()
        mtm = MultiTraceManager(bus=bus, config={})
        for i in range(1000):
            mtm.on_robot_xyz(['A10L', [i, [2*i, 0, 0]]])

        bus.reset_mock()
        mtm.on_trace_info({'B10R': [[1, 8]]})
        bus.publish.assert_called()
        # AssertionError: 20362 not less than 1000
        self.assertLess(len(str(bus.method_calls[-1][1])), 1000)

# vim: expandtab sw=4 ts=4
