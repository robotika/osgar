""""
    Records system information like CPU and RAM utilization or device temperatures.
"""

import sys
import psutil
# https://psutil.readthedocs.io/en/latest/#system-related-functions
from threading import Thread

class SystemMonitor:
    def __init__(self, config, bus):
        self.input_thread = Thread(target=self.run_input, daemon=True)
        self.bus = bus
        bus.register('cpu', 'ram', 'temp')
        self.sleep = config.get('sleep', 1)
        self.platform = sys.platform

    def start(self):
        self.input_thread.start()

    def join(self, timeout=None):
        self.input_thread.join(timeout=timeout)

    def run_input(self):
        while self.bus.is_alive():
            # current system-wide CPU utilization as a percentage
            cpu = psutil.cpu_percent(percpu=True)
            self.bus.publish('cpu', cpu)
            # system memory usage in percentage
            used_memory = psutil.virtual_memory().percent
            self.bus.publish('ram', used_memory)

            if self.platform == "linux":
                temp_raw = psutil.sensors_temperatures()
                assert "acpitz" in temp_raw, temp_raw
                temp = temp_raw["acpitz"][0].current  # Expects only one CPU.
                self.bus.publish('temp', temp)

            self.bus.sleep(self.sleep)

    def request_stop(self):
        self.bus.shutdown()
