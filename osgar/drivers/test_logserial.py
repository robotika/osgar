import unittest
from unittest.mock import patch, MagicMock

from osgar.drivers.logserial import LogSerial
from osgar.bus import BusHandler


class LogSerialTest(unittest.TestCase):

    def test_twoway_communication(self):
        with patch('osgar.drivers.logserial.serial.Serial') as mock:
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

    def test_timeout_config(self):
        with patch('osgar.drivers.logserial.serial.Serial', autospec=True) as mock:
            instance = mock.return_value
            bus = MagicMock()
            config = {'port':'COM13:', 'speed':4800, 'timeout':2.0}
            device = LogSerial(config=config, bus=bus)
            mock.assert_called_once_with('COM13:', 4800)
            self.assertAlmostEqual(instance.timeout, 2.0)

    def test_config_reset(self):
        with patch('osgar.drivers.logserial.serial.Serial', autospec=True) as mock:
            instance = mock.return_value
            bus = MagicMock()
            config = {'port':'COM10:', 'speed':9600, 'rtscts':True, 'reset':True}
            device = LogSerial(config=config, bus=bus)
            mock.assert_called_once_with('COM10:', 9600, rtscts=True)
            instance.setRTS.assert_called_once_with()
            instance.setDTR.assert_called_once_with(0)

# vim: expandtab sw=4 ts=4
