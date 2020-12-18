
import unittest
from unittest.mock import MagicMock, patch, call

from subt.multitrace import MultiTraceManager


class MultiTraceManagerTest(unittest.TestCase):

    def test_usage(self):
        bus = MagicMock()
        mtm = MultiTraceManager(bus=bus, config={})
        mtm.on_robot_xyz(['A150L', [505, [6.127597694909277, 2.640278491024048, 1.9188244676330934]]])
        mtm.on_robot_xyz(['A150L', [506, [5.07939188538461, 2.0835104023227147, 1.9305388336683675]]])
        bus.publish.assert_called()
        self.assertEqual(bus.method_calls[-1],
                         call.publish('trace_info', {'A150L': [505, 506]}))

        bus.reset_mock()
        mtm.on_trace_info({'B100R': [1, 10]})
        bus.publish.assert_called()
        self.assertEqual(bus.method_calls[-1],
                         call.publish('robot_trace', {'A150L': [[505, [6.127597694909277, 2.640278491024048, 1.9188244676330934]],
                                                                [506, [5.07939188538461, 2.0835104023227147, 1.9305388336683675]]]}))

        bus.reset_mock()
        mtm.on_trace_info({'A150L': [505, 506]})
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

# vim: expandtab sw=4 ts=4
