import unittest
from unittest.mock import patch, MagicMock, Mock

from osgar.drivers.logi2c import LogI2C
from osgar.bus import BusHandler


class LogI2CTest(unittest.TestCase):

    def test_send(self):
        with patch('osgar.drivers.logi2c.smbus.SMBus') as mock:
            instance = mock.return_value
            instance.read_i2c_block_data.return_value = 'X'

            logger = MagicMock()
            bus = BusHandler(logger, out={'i2c':[]})
            config = {'port':1}
            device = LogI2C(config=config, bus=bus)
            bus.queue.put((1, 2, [0x1E, 'R', 0x10, 1]))
            device.start()
            device.request_stop()
            device.join()
            instance.read_i2c_block_data.assert_called_once_with(0x1E, 0x10, 1)

    def test_io_exception(self):
        with patch('osgar.drivers.logi2c.smbus.SMBus') as mock:
            instance = mock.return_value
            instance.read_i2c_block_data = Mock(side_effect=OSError())

            logger = MagicMock()
            bus = BusHandler(logger, out={'i2c':[]})
            config = {'port':1}
            device = LogI2C(config=config, bus=bus)
            bus.queue.put((1, 2, [0x1E, 'R', 0x10, 1]))
            device.start()
            device.request_stop()
            device.join()
            instance.read_i2c_block_data.assert_called_with(0x1E, 0x10, 1)  # twice


# vim: expandtab sw=4 ts=4

