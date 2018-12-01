import unittest
from threading import Thread
from unittest.mock import patch, MagicMock, call
from xmlrpc.server import SimpleXMLRPCServer

from osgar.drivers.logrpc import LogServerProxy
from osgar.bus import BusHandler


def adder_function(x,y):
    return x + y


class MyTestRPCServer(Thread):
    def __init__(self):
        super().__init__()
        self.setDaemon(True)
        
        # Create server
        self.server = SimpleXMLRPCServer(("localhost", 8000))
        self.server.register_function(adder_function, 'add')
        self.start()

    def run(self):
        self.server.serve_forever()


class LogServerProxyTest(unittest.TestCase):

    def test_usage(self):
        server = MyTestRPCServer()
        bus = MagicMock()
        s = LogServerProxy(bus, 'http://localhost:8000')
        self.assertEqual(s.add(2, 3), 5)
        self.assertEqual(bus.publish.call_args_list, [
            call('send_request', b"<?xml version='1.0'?>\n<methodCall>\n<methodName>add</methodName>\n<params>\n<param>\n<value><int>2</int></value>\n</param>\n<param>\n<value><int>3</int></value>\n</param>\n</params>\n</methodCall>\n"),
            call('make_connection', 'localhost:8000'),
            call('raw', b"<?xml version='1.0'?>\n<methodResponse>\n<params>\n<param>\n<value><int>5</int></value>\n</param>\n</params>\n</methodResponse>\n")
            ])

# vim: expandtab sw=4 ts=4
