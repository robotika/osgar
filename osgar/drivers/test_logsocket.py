import unittest
from unittest.mock import patch, MagicMock, call
import time

from osgar.drivers.logsocket import LogTCPStaticIP as LogTCP, LogTCPDynamicIP, LogTCPServer, LogUDP, LogHTTP
from osgar.bus import Bus


class LogSocketTest(unittest.TestCase):

    def test_tcp_send(self):
        with patch('osgar.drivers.logsocket.socket.socket') as mock:
            instance = mock.return_value

            logger = MagicMock()
            logger.register = MagicMock(return_value=1)
            bus = Bus(logger)
            config = {'host': '192.168.2.23', 'port':2111}
            device = LogTCP(config=config, bus=bus.handle('tcp'))
            tester = bus.handle('tester')
            tester.register('raw')
            bus.connect('tester.raw', 'tcp.raw')
            tester.publish('raw', b'bin data')
            device.start()
            device.request_stop()
            device.join()
            instance.send.assert_called_once_with(b'bin data')


    def test_udp_send(self):
        with patch('osgar.drivers.logsocket.socket.socket') as mock:
            instance = mock.return_value

            logger = MagicMock()
            bus = Bus(logger)
            config = {'host': '192.168.2.23', 'port':2111}
            device = LogUDP(config=config, bus=bus.handle('udp'))
            tester = bus.handle('tester')
            tester.register('raw')
            bus.connect('tester.raw', 'udp.raw')
            tester.publish('raw', b'bin data')
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
            bus = Bus(logger)
            config = {'host': '192.168.1.2', 'port':8080}
            device = LogTCPServer(config=config, bus=bus.handle('tcp'))
            tester = bus.handle('tester')
            bus.connect('tcp.raw', 'tester.raw')

            device.start()
            device.request_stop()
            device.join()

            instance.listen.assert_called_once_with(1)
            instance.bind.assert_called_once_with(('192.168.1.2', 8080))

    def test_dynamic_tcp(self):
        with patch('osgar.drivers.logsocket.socket.socket') as mock:
            instance = mock.return_value

            logger = MagicMock()
            bus = Bus(logger)
            device = LogTCPDynamicIP(config={}, bus=bus.handle('tcpdyn'))
            tester = bus.handle('tester')
            tester.register('addr')
            bus.connect('tester.addr', 'tcpdyn.addr')
            tester.publish('addr', ['10.1.10.1', 8000])
            device.start()
            device.request_stop()
            device.join()

            instance.connect.assert_called_once_with(('10.1.10.1', 8000))

    def test_not_started_dynamic_tcp(self):
        with patch('osgar.drivers.logsocket.socket.socket') as mock:
            instance = mock.return_value

            logger = MagicMock()
            bus = Bus(logger)
            config = {}
            device = LogTCPDynamicIP(config=config, bus=bus.handle('tcp'))
            device.start()
            device.request_stop()
            device.join()

            instance.connect.assert_not_called()

    def test_dynamic_tcp_called_twice(self):
        with patch('osgar.drivers.logsocket.socket.socket') as mock:
            instance = mock.return_value

            logger = MagicMock()
            bus = Bus(logger)
            device = LogTCPDynamicIP(config={}, bus=bus.handle('tcpdyn'))
            tester = bus.handle('tester')
            tester.register('addr')
            bus.connect('tester.addr', 'tcpdyn.addr')
            tester.publish('addr', ['10.1.10.1', 8000])
            tester.publish('addr', ['192.168.1.31', 8010])
            device.start()
            tester.sleep(0.1)
            device.request_stop()
            device.join()

            self.assertEqual(instance.connect.call_args_list, [
                    call(('10.1.10.1', 8000)),
                    call(('192.168.1.31', 8010))
                ])

    def test_http_sleep(self):
        # reported as bug for IP camera running at full speed
        with patch('osgar.drivers.logsocket.urllib.request.urlopen') as mock:
            instance = mock.return_value
            instance.__enter__.return_value.read = MagicMock(return_value=b'123')
            logger = MagicMock()
            logger.register = MagicMock(return_value=1)
            logger.write = MagicMock(return_value=123)
            bus = Bus(logger)
            config = {
              "url": "http://192.168.0.99/img.jpg",
              "sleep": 0.1,
              "timeout": 1.0
            }
            device = LogHTTP(config=config, bus=bus.handle('http'))
            tester = bus.handle('tester')
            bus.connect('http.raw', 'tester.raw')

            device.start()
            data = tester.listen()
            self.assertEqual(data, (123, 'raw', b'123'))
            device.request_stop()
            device.join()
            self.assertEqual(
                    len(instance.__enter__.return_value.read.call_args_list),
                    1)  # it should be just one call and sleep

# vim: expandtab sw=4 ts=4
