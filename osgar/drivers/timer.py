"""
  Timer event trigger
"""

from threading import Thread
import time

from osgar.bus import BusShutdownException


class Timer(Thread):
    def __init__(self, config, bus):
        Thread.__init__(self)
        self.setDaemon(True)
        self.bus = bus
        self.sleep_time = config['sleep']

    def run(self):
        try:
            count = 0
            while self.bus.is_alive():
                self.bus.publish('tick', count)
                time.sleep(self.sleep_time)
                count += 1
        except BusShutdownException:
            pass

    def request_stop(self):
        self.bus.shutdown()

# vim: expandtab sw=4 ts=4
