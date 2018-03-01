import unittest
from unittest.mock import patch, MagicMock

from drivers.logserial import LogSerial
from drivers.bus import BusHandler


class LogSerialTest(unittest.TestCase):

    def test_twoway_communication(self):
        with patch('drivers.logserial.serial.Serial') as mock:
            instance = mock.return_value

            logger = MagicMock()
            bus = BusHandler(logger)
            config = {'port':'COM13:', 'speed':4800}
            device = LogSerial(config=config, bus=bus)
            bus.queue.put((1, 2, b'bin data'))
            device.start()
            device.request_stop()
            device.join()
            instance.write.assert_called_once_with(b'bin data')


# vim: expandtab sw=4 ts=4
