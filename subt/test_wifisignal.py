import unittest
from unittest.mock import MagicMock, patch
from time import sleep

from subt.wifisignal import WifiSignal
from osgar.bus import BusShutdownException


class WifiSignalTest(unittest.TestCase):

    def test_usage(self):
        with patch('subt.wifisignal.wifi_scan') as p:
            bus = MagicMock()
            ws = WifiSignal(config={}, bus=bus)
            bus.is_alive = MagicMock(return_value=True)
            ws.start()
            sleep(0.1)
            bus.is_alive = MagicMock(return_value=False)
            ws.join()

# vim: expandtab sw=4 ts=4

