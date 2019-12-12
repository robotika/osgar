from threading import Thread


class Timer:
    def __init__(self, config, bus):
        self.bus = bus
        bus.register('tick')
        self.sleep = config['sleep']
        self.thread = Thread(target=self.run)

    def start(self):
        self.thread.start()

    def join(self, timeout=None):
        self.thread.join(timeout=timeout)

    def run(self):
        while self.bus.is_alive():
            self.bus.publish('tick', None)
            self.bus.sleep(self.sleep)

    def request_stop(self):
        self.bus.shutdown()
