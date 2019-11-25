import unittest
from unittest.mock import MagicMock
import time

from osgar.drivers.logzeromq import LogZeroMQ
from osgar.bus import BusShutdownException


class LogZeroMQTest(unittest.TestCase):
    def test_recv_timeout(self):
        config = {
            'mode': 'PULL',
            'endpoint': 'tcp://localhost:5555',
            'timeout': 0.1
        }
        bus = MagicMock()
        bus.is_alive = MagicMock(return_value=True)
        node = LogZeroMQ(config, bus)
        node.start()
        time.sleep(0.01)  # give it a chance to start
        node.request_stop()
        bus.is_alive = MagicMock(return_value=False)  # supplement mock request_stop()
        node.join()

    def test_send_timeout(self):
        config = {
            'mode': 'PUSH',
            'endpoint': 'tcp://localhost:5556',
            'timeout': 0.1
        }
        bus = MagicMock()
        bus.listen = MagicMock(return_value=(1, 2, b'data'))
        bus.is_alive = MagicMock(return_value=True)
        node = LogZeroMQ(config, bus)
        node.start()
        time.sleep(0.01)  # give it a chance to start
        node.request_stop()
        bus.listen = MagicMock(side_effect=BusShutdownException())
        bus.is_alive = MagicMock(return_value=False)  # supplement mock request_stop()
        node.join()

# vim: expandtab sw=4 ts=4
