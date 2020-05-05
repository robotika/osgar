import unittest
from unittest.mock import MagicMock
import threading
from threading import Thread
import os
from xmlrpc.server import SimpleXMLRPCServer
from xmlrpc.client import ServerProxy

from osgar.drivers.rosproxy import ROSProxy, prefix4BytesLen
from osgar.bus import Bus


# TODO refactor into common testing code
# https://stackoverflow.com/questions/12484175/make-python-unittest-fail-on-exception-from-any-thread
class GlobalExceptionWatcher:

    def __init__(self):
        self._excepthook_name = 'excepthook' if getattr(threading, 'excepthook', None) is not None else '_format_exc'

    def _store_excepthook(self):
        '''
        Uses as an exception handlers which stores any uncaught exceptions.
        '''
        formated_exc = self.__org_hook()
        self._exceptions.append(formated_exc)
        return formated_exc

    def __enter__(self):
        '''
        Register us to the hook.
        '''
        self._exceptions = []
        self.__org_hook = getattr(threading, self._excepthook_name)
        setattr(threading, self._excepthook_name, self._store_excepthook)

    def __exit__(self, type, value, traceback):
        '''
        Remove us from the hook, assure no exception were thrown.
        '''
        setattr(threading, self._excepthook_name, self.__org_hook)
        if len(self._exceptions) != 0:
            tracebacks = os.linesep.join(self._exceptions)
            raise Exception('Exceptions in other threads: %s' % tracebacks)


def getSystemState(path):
    print("RECEIVED", path)
    return (1, 0, ([], 0, 0))


def registerPublisher(a, b, c, d):
    return (1, 0, [])


def registerSubscriber(a, b, c, d):
    # re-use master to pretend to be also a node ...
    return (1, 0, ['http://127.0.0.1:11311'])


def requestTopic(a, b, c):
    return (1, 0, [1, 2, 3])


class DummyROSMaster(Thread):
    def __init__(self, host_port_addr):
        Thread.__init__(self)
        self.setDaemon(True)
        self.server = SimpleXMLRPCServer(host_port_addr)
        print('Listening on %s:%d ...' % host_port_addr)
        self.server.register_function(getSystemState, 'getSystemState')
        self.server.register_function(registerPublisher, 'registerPublisher')
        self.server.register_function(registerSubscriber, 'registerSubscriber')

        # fake calls for "real" ROS node
        self.server.register_function(requestTopic, 'requestTopic')
        self.server.register_function(self.request_shutdown, 'request_shutdown')
        self.quit = False
        self.host_port_addr = host_port_addr
        self.start()

    def request_shutdown(self):
        self.quit = True
        return True

    def run( self ):
        while not self.quit:
            self.server.handle_request()

    def shutdown(self):
        url = "http://{}:{}".format(*self.host_port_addr)
        s = ServerProxy(url)
        s.request_shutdown()
        self.join(timeout=1)
        if self.is_alive():
            raise RuntimeError("DummyROSMaster failed to shutdown.")
        self.server.server_close()


class ROSProxyTest(unittest.TestCase):

    def test_usage(self):
        logger = MagicMock()
        bus = Bus(logger)
        config = {
                'ros_master_uri': 'http://127.0.0.1:11311',
                'ros_client_uri': 'http://127.0.0.1:8000',
                'topic': '/hello',
                'topic_type': 'std_msgs/String',

                'subscribe': [
                        ['/imu/data', 'std_msgs/Imu', 'imu_data']
                    ]
                }

        master = DummyROSMaster(('127.0.0.1', 11311))
        proxy = ROSProxy(config=config, bus=bus.handle('ros'))
        with GlobalExceptionWatcher():
            proxy.start()
            proxy.request_stop()
            proxy.join(timeout=1)
            self.assertFalse(proxy.is_alive())
        master.shutdown()

    def test_prefix4BytesLen(self):
        self.assertEqual(prefix4BytesLen('ahoj'), bytes([4, 0, 0, 0]) + b'ahoj')
        self.assertEqual(prefix4BytesLen(b'\x01bin'), bytes([4, 0, 0, 0]) + b'\x01bin')

# vim: expandtab sw=4 ts=4
