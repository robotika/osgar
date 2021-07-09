import unittest
from unittest.mock import MagicMock
import sys
sys.modules['psutil'] = MagicMock()
from osgar.drivers.system_monitor import get_timestamp, SystemMonitor


class TestSystemMonitor(unittest.TestCase):

    def test_get_timestamp(self):
        msg = b"[330199.830015] wlp5s0: associated"
        msg2 = b"[   14.850440] IPv6: ADDRCONF(NETDEV_CHANGE): wlp5s0: link becomes ready"
        timestamp = get_timestamp(msg)
        timestamp2 = get_timestamp(msg2)
        self.assertEqual(timestamp, 330199.830015)
        self.assertEqual(timestamp2, 14.850440)

    def test_process_dmesg(self):
        s = SystemMonitor(bus = MagicMock(), config = {})
        s.last_dmesg_time = 330199.830015
        msg  = [b"[330199.830015] wlp5s0: associated",
                b"[330199.930015] wlp5s0: some info",
                b"[330200.607397] IPv6: ADDRCONF(NETDEV_CHANGE): wlp5s0: link becomes ready",
                b""]
        new_msg = "[330199.930015] wlp5s0: some info\n" \
                  "[330200.607397] IPv6: ADDRCONF(NETDEV_CHANGE): wlp5s0: link becomes ready\n"
        dmesg = s.process_dmesg(msg)
        self.assertEqual(dmesg, new_msg)

if __name__ == '__main__':
    unittest.main()
