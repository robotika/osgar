import unittest
from unittest.mock import MagicMock
import time

import zmq

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
        bus.name = "test_recv_timeout"
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

    def test_req(self):
        config = {
            'mode': 'REQ',
            'endpoint': 'tcp://localhost:5557',
            'timeout': 0.1
        }
        bus = MagicMock()
        bus.listen = MagicMock(return_value=(1, 2, 'set_brakes on\n'))
        bus.is_alive = MagicMock(return_value=True)
        node = LogZeroMQ(config, bus)
        node.start()
        time.sleep(0.1)  # give it a chance to start
        node.request_stop()
        bus.listen = MagicMock(side_effect=BusShutdownException())
        bus.is_alive = MagicMock(return_value=False)  # supplement mock request_stop()

        # we have to readout the request otherwise the sender context.term() will never terminate
        context = zmq.Context()
        socket = context.socket(zmq.REP)
        socket.RCVTIMEO = 1000
        socket.bind('tcp://127.0.0.1:5557')
        msg = socket.recv()
        node.join()
        self.assertEqual(msg, b'set_brakes on\n')
        bus.publish.assert_called_with('timeout', True)

# vim: expandtab sw=4 ts=4
