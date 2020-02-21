import unittest
from unittest.mock import MagicMock
from datetime import timedelta

from osgar.bus import Bus

from subt.scan_mixer import ScanMixer


class ScanMixerTest(unittest.TestCase):

    def test_usage(self):
        bus = Bus(MagicMock())
        tester = bus.handle('tester')
        tester.register("scan", "rs_scan")
        mixer = ScanMixer(config={}, bus=bus.handle('mixer'))
        bus.connect("tester.scan", "mixer.scan")
        bus.connect("tester.rs_scan", "mixer.rs_scan")
        bus.connect("mixer.scan", "tester.mixed")
        mixer.start()
        scan = [10, 10, 10, 10]
        tester.publish("scan", scan)
        _, channel, data = tester.listen()
        self.assertEqual(data, scan)
        mixer.request_stop()
        mixer.join()

    def test_mixing(self):
        log = MagicMock()
        log.write.return_value = timedelta()
        bus = Bus(log)
        tester = bus.handle('tester')
        tester.register("scan", "rs_scan")
        mixer = ScanMixer(config={}, bus=bus.handle('mixer'))
        bus.connect("tester.scan", "mixer.scan")
        bus.connect("tester.rs_scan", "mixer.rs_scan")
        bus.connect("mixer.scan", "tester.mixed")
        mixer.start()
        scan = 4*[10]
        rs_scan = 3*[8]
        tester.publish("rs_scan", rs_scan)
        tester.publish("scan", scan)
        _, channel, data = tester.listen()
        self.assertEqual(data, 4*[8])
        mixer.request_stop()
        mixer.join()

    def test_infinity(self):
        log = MagicMock()
        log.write.return_value = timedelta()
        bus = Bus(log)
        tester = bus.handle('tester')
        tester.register("scan", "rs_scan")
        mixer = ScanMixer(config={}, bus=bus.handle('mixer'))
        bus.connect("tester.scan", "mixer.scan")
        bus.connect("tester.rs_scan", "mixer.rs_scan")
        bus.connect("mixer.scan", "tester.mixed")

        mixer.start()
        scan = 3*[10]+[0]
        rs_scan = 3*[8]
        tester.publish("rs_scan", rs_scan)
        tester.publish("scan", scan)
        _, channel, data = tester.listen()
        self.assertEqual(data, 4*[8])

        scan = 4*[10]
        rs_scan = 2*[8]+[0]
        tester.publish("rs_scan", rs_scan)
        tester.publish("scan", scan)
        _, channel, data = tester.listen()
        self.assertEqual(data, 3*[8]+[10])

        mixer.request_stop()
        mixer.join()

    def test_old_rs_scan(self):
        log = MagicMock()
        log.write.return_value = timedelta()
        bus = Bus(log)
        tester = bus.handle('tester')
        tester.register("scan", "rs_scan")
        mixer = ScanMixer(config={}, bus=bus.handle('mixer'))
        bus.connect("tester.scan", "mixer.scan")
        bus.connect("tester.rs_scan", "mixer.rs_scan")
        bus.connect("mixer.scan", "tester.mixed")

        mixer.start()
        scan = 3*[10]+[0]
        rs_scan = 3*[8]
        tester.publish("rs_scan", rs_scan)
        log.write.return_value = timedelta(seconds=1)
        tester.publish("scan", scan)
        _, channel, data = tester.listen()
        self.assertEqual(data, 3*[10]+[0])

        mixer.request_stop()
        mixer.join()

# vim: expandtab sw=4 ts=4
