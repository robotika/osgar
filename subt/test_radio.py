import unittest
import datetime
from unittest.mock import MagicMock, patch, call

from subt.radio import Radio
from osgar.bus import Bus
from osgar.lib.serialize import serialize


class RadioTest(unittest.TestCase):

    def test_on_radio(self):
        # virtual world
        data = [b'A0F150L', serialize([13, 'artf', ['TYPE_RESCUE_RANDY', 15982, 104845, 3080]])]
        bus = MagicMock()
        dev = Radio(bus=bus, config={})
        bus.reset_mock()
        dev.on_radio(data)
        bus.publish.assert_called()

        data = [b'A0F150L', serialize([0, 'pose2d', [11896, 7018, -11886]])]
        bus.reset_mock()
        dev.on_radio(data)
        bus.publish.assert_not_called()

    def test_on_breadcrumb(self):
        data = [29.929257253892082, -1.5821685677914703, 1.575092509709292]  # xyz location of newly deployed breadcrumb
        bus = MagicMock()
        dev = Radio(bus=bus, config={})
        bus.reset_mock()
        dev.on_breadcrumb(data)
        bus.publish.assert_called()
        self.assertEqual(bus.method_calls[0],
                         call.publish('radio', serialize([0, 'breadcrumb', [29.929257253892082, -1.5821685677914703, 1.575092509709292]])))

        new_msg = serialize([0, 'breadcrumb', [29.929257253892082, -1.5821685677914703, 1.575092509709292]])
        bus.reset_mock()
        dev.on_radio([b'A1300L', new_msg])
        bus.publish.assert_called()

    def test_msg_pack(self):
        bus = MagicMock()
        dev = Radio(bus=bus, config={})
        bus.reset_mock()
        dev.send_data('dummy_artf', [1, 2, 3])
        bus.publish.assert_called()
        self.assertEqual(bus.method_calls[-1],
                         call.publish('radio', serialize([0, 'dummy_artf', [1, 2, 3]])))
        dev.send_data('dummy_artf', [1, 2, 3])
        self.assertEqual(bus.method_calls[-1],
                         call.publish('radio', serialize([1, 'dummy_artf', [1, 2, 3]])))

# vim: expandtab sw=4 ts=4
