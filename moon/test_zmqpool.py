import unittest
from unittest.mock import MagicMock
import time
from threading import Thread

import zmq

from moon.zmqpool import ZMQPool
from osgar.bus import BusShutdownException


class ZMQPoolTest(unittest.TestCase):

    class DummyROSServer(Thread):
        def __init__(self):
            super().__init__()
            self.msg = b''
            self.stop_requested = False
            
        def stop(self):
            self.stop_requested = True
            
        def run(self):
            context = zmq.Context()
            socket = context.socket(zmq.REP)
            socket.RCVTIMEO = 1000
            socket.bind('tcp://127.0.0.1:5557')
            while not self.stop_requested:
                try:
                    self.msg = socket.recv()
                    socket.send_string("OK")
                except zmq.Again:
                    pass
                
        def get_message(self):
            return self.msg
        
    def test_req(self):
        config = {
            'mode': 'REQ',
            'endpoint': 'tcp://localhost:5557',
            'timeout': 0.1
        }
        bus = MagicMock()
        bus.listen = MagicMock(return_value=(1, 2, ['abcdef', 'set_brakes on']))
        bus.is_alive = MagicMock(return_value=True)
        node = ZMQPool(config, bus)

        t = self.DummyROSServer()
        t.start()
        node.start()
        time.sleep(0.1)  # give it a chance to start
        node.request_stop()
        bus.listen = MagicMock(side_effect=BusShutdownException())
        bus.is_alive = MagicMock(return_value=False)  # supplement mock request_stop()
        node.join()
        t.stop()
        msg = t.get_message()
        t.join()

        self.assertEqual(msg, b'set_brakes on')
        bus.publish.assert_called_with('response', ['abcdef', 'OK'])

# vim: expandtab sw=4 ts=4
