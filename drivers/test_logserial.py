import unittest
from unittest.mock import MagicMock

from drivers.logserial import LogSerialOut
from drivers.bus import BusHandler


class LogSerialTest(unittest.TestCase):

    def test_output(self):
        logger = MagicMock()
        bus = BusHandler(logger)
        serial = MagicMock()
        device = LogSerialOut(config=None, bus=bus, com=serial)
        bus.queue.put((1, 2, b'bin data'))
        device.start()
        device.request_stop()
        device.join()
        serial.write.assert_called_once_with(b'bin data')


# vim: expandtab sw=4 ts=4
