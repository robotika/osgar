import unittest
from unittest.mock import patch, MagicMock, call
import time

from osgar.drivers.logsocket import LogTCPStaticIP as LogTCP, LogTCPDynamicIP, LogTCPServer, LogUDP
from osgar.bus import BusHandler


class LogSocketTest(unittest.TestCase):

    def test_tcp_send(self):
        with patch('osgar.drivers.logsocket.socket.socket') as mock:
            instance = mock.return_value

            logger = MagicMock()
            logger.register = MagicMock(return_value=1)
            bus = BusHandler(logger)
            config = {'host': '192.168.2.23', 'port':2111}
            device = LogTCP(config=config, bus=bus)
            bus.queue.put((1, 'raw', b'bin data'))
            device.start()
            device.request_stop()
            device.join()
            instance.send.assert_called_once_with(b'bin data')


    def test_udp_send(self):
        with patch('osgar.drivers.logsocket.socket.socket') as mock:
            instance = mock.return_value

            logger = MagicMock()
            bus = BusHandler(logger)
            config = {'host': '192.168.2.23', 'port':2111}
            device = LogUDP(config=config, bus=bus)
            bus.queue.put((1, 'raw', b'bin data'))
            device.start()
            device.request_stop()
            device.join()
            instance.sendto.assert_called_once_with(
                    b'bin data', ('192.168.2.23', 2111))

    def test_tcp_server(self):
        with patch('osgar.drivers.logsocket.socket.socket') as mock:
            instance = mock.return_value
            instance.accept = MagicMock(return_value=('127.0.0.1', 1234))
            instance.recv = MagicMock(return_value=b'some bin data')

            logger = MagicMock()
            bus = BusHandler(logger)
            config = {'host': '192.168.1.2', 'port':8080}
            device = LogTCPServer(config=config, bus=bus)
            device.start()
            device.request_stop()
            device.join()

            instance.listen.assert_called_once_with(1)
            instance.bind.assert_called_once_with(('192.168.1.2', 8080))

    def test_dynamic_tcp(self):
        with patch('osgar.drivers.logsocket.socket.socket') as mock:
            instance = mock.return_value

            logger = MagicMock()
            bus = BusHandler(logger)
            bus.queue.put((1, 'addr', ['10.1.10.1', 8000]))
            config = {}
            device = LogTCPDynamicIP(config=config, bus=bus)
            device.start()
            time.sleep(0.1)
            device.request_stop()
            device.join()

            instance.connect.assert_called_once_with(('10.1.10.1', 8000))

    def test_not_started_dynamic_tcp(self):
        with patch('osgar.drivers.logsocket.socket.socket') as mock:
            instance = mock.return_value

            logger = MagicMock()
            bus = BusHandler(logger)
            config = {}
            device = LogTCPDynamicIP(config=config, bus=bus)
            device.start()
            time.sleep(0.1)
            device.request_stop()
            device.join()

            instance.connect.assert_not_called()

    def test_dynamic_tcp_called_twice(self):
        with patch('osgar.drivers.logsocket.socket.socket') as mock:
            instance = mock.return_value

            logger = MagicMock()
            bus = BusHandler(logger)
            bus.queue.put((1, 'addr', ['10.1.10.1', 8000]))
            bus.queue.put((42, 'addr', ['192.168.1.31', 8010]))
            config = {}
            device = LogTCPDynamicIP(config=config, bus=bus)
            device.start()
            time.sleep(0.1)
            device.request_stop()
            device.join()

            self.assertEqual(instance.connect.call_args_list, [
                    call(('10.1.10.1', 8000)),
                    call(('192.168.1.31', 8010))
                ])

# vim: expandtab sw=4 ts=4
