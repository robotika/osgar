"""
  Log standard input
"""
import sys
from threading import Thread

from osgar.bus import BusShutdownException


class LogStdIn:
    def __init__(self, config, bus):
        self.input_thread = Thread(target=self.run_input, daemon=True)
        self.bus = bus

        self.linenum = config.get('linenum', False)
        self.sleep = config.get('sleep')

    def start(self):
        self.input_thread.start()

    def join(self, timeout=None):
        self.input_thread.join(timeout=timeout)

    def run_input(self):
        for i, line in enumerate(sys.stdin):
            if self.linenum:
                s = str(i) + ': ' + line.strip()
            self.bus.publish('line', s)
            if self.sleep is not None:
                self.bus.sleep(self.sleep)
            if not self.bus.is_alive():
                break
        self.request_stop()  # maybe conditional based on config?

    def request_stop(self):
        self.bus.shutdown()

# vim: expandtab sw=4 ts=4
