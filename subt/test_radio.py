import unittest
import datetime
from unittest.mock import MagicMock, patch, call

from subt.radio import Radio
from osgar.bus import Bus


class RadioTest(unittest.TestCase):

    def test_on_radio(self):
        # virtual world
        data = [b'A0F150L', b"['TYPE_RESCUE_RANDY', 15982, 104845, 3080]\n"]
        bus = MagicMock()
        dev = Radio(bus=bus, config={})
        bus.reset_mock()
        dev.on_radio(data)
        bus.publish.assert_called()

        data = [b'A0F150L', b'[11896, 7018, -11886]\n']
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
                         call.publish('radio', b"{'breadcrumb': [29.929257253892082, -1.5821685677914703, 1.575092509709292]}\n"))

        new_msg = b"{'breadcrumb': [29.929257253892082, -1.5821685677914703, 1.575092509709292]}\n"
        bus.reset_mock()
        dev.on_radio([b'A1300L', new_msg])
        bus.publish.assert_called()

# vim: expandtab sw=4 ts=4
