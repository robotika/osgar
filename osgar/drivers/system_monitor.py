""""
    Records system information like CPU and RAM utilization or device temperatures.
"""

import psutil
# https://psutil.readthedocs.io/en/latest/#system-related-functions
from threading import Thread

class SystemMonitor:
    def __init__(self, config, bus):
        self.input_thread = Thread(target=self.run_input, daemon=True)
        self.bus = bus
        bus.register('cpu', 'ram', 'temp')
        self.sleep = config.get('sleep', 1)

    def start(self):
        self.input_thread.start()

    def join(self, timeout=None):
        self.input_thread.join(timeout=timeout)

    def run_input(self):
        while self.bus.is_alive():
            # current system-wide CPU utilization as a percentage
            cpu = psutil.cpu_percent(percpu=True)
            if cpu:
                self.bus.publish('cpu', cpu)
            # system memory usage in percentage
            used_memory = psutil.virtual_memory().percent
            if used_memory:
                self.bus.publish('ram', used_memory)
            # Probably it is not working on MS Windows. TODO test it.
            temp_raw = psutil.sensors_temperatures()
            if temp_raw:
                assert "acpitz" in temp_raw, temp_raw
                temp = temp_raw["acpitz"][0].current  # Expects only one CPU.
                self.bus.publish('temp', temp)

            self.bus.sleep(self.sleep)

    def request_stop(self):
        self.bus.shutdown()
