import subprocess

from osgar.node import Node


def wifi_scan():
    with subprocess.Popen('sudo iwlist wlan0 scanning', shell=True,
            stdout=subprocess.PIPE) as cmd:
        wifiList = []
        for line in cmd.stdout:
            if 'Quality' in str(line):
                signalStrength = int(str(line).split("=")[2].split('dBm')[0])
            elif 'ESSID' in str(line):
                essid = str(line).split('"')[1]
                wifiList.append([essid,signalStrength])
        return wifiList


class WifiSignal(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        self.sleep_time = config.get('sleep', 1.0)
        # TODO ssid
        # search SSID

    def run(self):
        while self.is_alive():
            wifi_list = wifi_scan()
            now = self.publish("wifiscan", wifi_list)
            self.sleep(self.sleep_time)

if __name__ == "__main__":
    import time
    while True:
        result = wifi_scan()
        print(result)
        time.sleep(1)

# vim: expandtab sw=4 ts=4

