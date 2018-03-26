import unittest
from unittest.mock import patch, MagicMock

from osgar.drivers.logsocket import LogTCP
from osgar.drivers.bus import BusHandler


class LogSocketTest(unittest.TestCase):

    def test_tcp_send(self):
        with patch('osgar.drivers.logsocket.socket.socket') as mock:
            instance = mock.return_value

            logger = MagicMock()
            bus = BusHandler(logger)
            config = {'host': '192.168.2.23', 'port':2111}
            device = LogTCP(config=config, bus=bus)
            bus.queue.put((1, 2, b'bin data'))
            device.start()
            device.request_stop()
            device.join()
            instance.send.assert_called_once_with(b'bin data')

# vim: expandtab sw=4 ts=4
