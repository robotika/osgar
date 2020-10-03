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

# vim: expandtab sw=4 ts=4
