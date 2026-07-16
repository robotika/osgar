"""
  Timer
"""
from threading import Thread

from osgar.bus import BusShutdownException


class MyTimer(Thread):
    def __init__(self, config, bus):
        Thread.__init__(self)
        self.setDaemon(True)
        self.bus = bus
        self.sleep_time = config['sleep']

    def run(self):
        try:
            while self.bus.is_alive():
                self.bus.publish('tick', None)
                self.bus.sleep(self.sleep_time)

        except BusShutdownException:
            pass

    def request_stop(self):
        self.bus.shutdown()

# vim: expandtab sw=4 ts=4

