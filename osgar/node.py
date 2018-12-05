"""
  Node - processing unit
"""
from threading import Thread

from osgar.bus import BusShutdownException


class Node(Thread):
    """
       Parent classs for all processing "nodes".
    """
    def __init__(self, config, bus):
        super().__init__()
        self.setDaemon(True)
        self.bus = bus
        self.time = None

    def publish(self, channel, data):
        return self.bus.publish(channel, data)

    def listen(self):
        return self.bus.listen()

    def sleep(self, secs):
        self.bus.sleep(secs)

    def is_alive(self):
        return self.bus.is_alive()

    def update(self):
        timestamp, channel, data = self.bus.listen()
        self.time = timestamp
        setattr(self, channel, data)
        return channel

    def run(self):
        try:
            while True:
                self.update()
        except BusShutdownException:
            pass

    def request_stop(self):
        self.bus.shutdown()

# vim: expandtab sw=4 ts=4
