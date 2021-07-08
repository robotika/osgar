""""
    Records system information like CPU and RAM utilization or device temperatures.
"""

import sys
import subprocess
import psutil
# https://psutil.readthedocs.io/en/latest/#system-related-functions
from threading import Thread


def get_timestamp(msg):
    return float(msg.split()[0][1:-1])

class SystemMonitor:
    def __init__(self, config, bus):
        self.input_thread = Thread(target=self.run_input, daemon=True)
        self.bus = bus
        bus.register('cpu', 'ram', 'temp')
        self.sleep = config.get('sleep', 1)
        self.log_dmesg = config.get('dmesg', False)
        self.last_dmesg_time = None
        if self.log_dmesg:
            bus.register('dmesg')
        self.platform = sys.platform
        self.first_meas = True

    def process_dmesg(self, dmesg_all):
        ret = ""
        if self.last_dmesg_time is None:
            self.last_dmesg_time = get_timestamp(dmesg_all[-2])  # the last line is empty
            return None
        new_last_time = get_timestamp(dmesg_all[-2])
        if self.last_dmesg_time == new_last_time:
            return None

        ii = -2
        while True:
            msg = dmesg_all[ii].decode("utf-8")
            ret = msg + "\n" + ret
            ii -= 1
            timestamp = get_timestamp(dmesg_all[ii])
            if self.last_dmesg_time == timestamp:
                self.last_dmesg_time = new_last_time
                return ret

    def start(self):
        self.input_thread.start()

    def join(self, timeout=None):
        self.input_thread.join(timeout=timeout)

    def run_input(self):
        while self.bus.is_alive():
            # current system-wide CPU utilization as a percentage
            cpu = psutil.cpu_percent(percpu=True)
            if not self.first_meas:
                self.bus.publish('cpu', cpu)
            else:
                self.first_meas = False
            # system memory usage in percentage
            used_memory = psutil.virtual_memory().percent
            self.bus.publish('ram', used_memory)

            if self.platform == "linux":
                temp_raw = psutil.sensors_temperatures()
                if "acpitz" in temp_raw:
                    temp = temp_raw["acpitz"][0].current  # Expects only one CPU.
                elif "k10temp" in temp_raw:
                    temp = temp_raw["k10temp"][0].current  # Expects only one CPU.
                else:
                    temp = None  # Unsupported case.
                if temp is not None:
                    self.bus.publish('temp', temp)
                proc_dmesg = subprocess.Popen(["dmesg"], stdout=subprocess.PIPE)
                dmesg_all = proc_dmesg.stdout.read().split(b"\n")
                dmesg = self.process_dmesg(dmesg_all)
                if dmesg is not None:
                    self.bus.publish('dmesg', dmesg)

            self.bus.sleep(self.sleep)

    def request_stop(self):
        self.bus.shutdown()
