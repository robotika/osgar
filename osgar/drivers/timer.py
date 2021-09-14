from threading import Thread
import time


class Timer:
    def __init__(self, config, bus):
        self.bus = bus
        bus.register('tick')
        self.sleep = config['sleep']
        self.thread = Thread(target=self.run)
        self.time_zero = time.monotonic()

    def start(self):
        self.thread.start()

    def join(self, timeout=None):
        self.thread.join(timeout=timeout)

    def run(self):
        while self.bus.is_alive():
            self.bus.publish('tick', time.monotonic() - self.time_zero)
            self.bus.sleep(self.sleep)

    def request_stop(self):
        self.bus.shutdown()
