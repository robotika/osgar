import unittest
import time
from queue import Queue
from unittest.mock import MagicMock

from osgar.bus import BusShutdownException
from osgar.drivers.johndeere import JohnDeere, CAN_ID_ENCODERS


class DummyBus:
    def __init__(self):
        self.queue = Queue()
        self.last_published = None

    def publish(self, channel, data):
        self.last_published = (channel, data)

    def listen(self):
        packet = self.queue.get()
        if packet is None:
            raise BusShutdownException()
        timestamp, channel, data = packet
        return timestamp, channel, data

    def shutdown(self):
        self.queue.put(None)


class JohnDeereTest(unittest.TestCase):

    def test_usage(self):
        bus = DummyBus()
        jd = JohnDeere(config={}, bus=bus)
        jd.start()
        msg = [(CAN_ID_ENCODERS >> 3)&0xFF, (CAN_ID_ENCODERS & 0x7) << 5, 0, 0, 0, 0]
        bus.queue.put((0, 'can', bytes(msg)))
        # make sure jd process input message
        time.sleep(0.2)
        jd.request_stop()
        jd.join()
        self.assertEqual(jd.bus.last_published, ('encoders', [0, 0]))

# vim: expandtab sw=4 ts=4
