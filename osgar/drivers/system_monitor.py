""""
    Records system information like CPU and RAM utilization or device temperatures.
"""

import psutil
from threading import Thread

class SystemMonitor:
    def __init__(self, config, bus):
        self.input_thread = Thread(target=self.run_input, daemon=True)
        self.bus = bus
        bus.register('cpu', 'ram', 'net_raw', 'temp_raw')
        self.sleep = config.get('sleep', 1)

    def start(self):
        self.input_thread.start()

    def join(self, timeout=None):
        self.input_thread.join(timeout=timeout)

    def run_input(self):
        while self.bus.is_alive():
            cpu = psutil.cpu_freq(percpu=True)
            used_memory = psutil.virtual_memory().percent
            net_raw = psutil.net_io_counters(pernic=True)
            temp_raw = psutil.sensors_temperatures()
            self.bus.publish('cpu', cpu)
            self.bus.publish('ram', used_memory)
            self.bus.publish('net_raw', net_raw)
            self.bus.publish('temp_raw', temp_raw)

            self.bus.sleep(self.sleep)

    def request_stop(self):
        self.bus.shutdown()
