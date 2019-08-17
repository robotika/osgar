import subprocess

def wifiScan():
    cmd = subprocess.Popen('sudo iwlist wlan0 scanning', shell=True,
                           stdout=subprocess.PIPE)
    wifiList = []
    for line in cmd.stdout:
        if 'Quality' in str(line):
            signalStrength = int(str(line).split("=")[2].split('dBm')[0])
        elif 'ESSID' in str(line):
            essid = str(line).split('"')[1]
            wifiList.append([essid,signalStrength])
    return wifiList

if __name__ == "__main__":
    import time
    while True:
        result = wifiScan()
        print(result)
        time.sleep(1)

