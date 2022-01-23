"""
  OSGAR Pozyx (utrawide-band trilateration) wrapper
"""
import itertools

import pypozyx

from osgar.node import Node
from osgar.bus import BusShutdownException


class Pozyx(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('range', 'settings')
        serial_port = config['port']
        self.sleep_sec = config.get("sleep")
        self.devices = [int(x, 16) for x in config.get('devices', [])]  # unfortunately JSON does not support hex
        self.devices.append(None)  # extra range to the base (must be last, 2nd param)
        self.pozyx = pypozyx.PozyxSerial(serial_port)
        self.verbose = False

    def get_settings(self):
        settings = pypozyx.UWBSettings()
        for remote_id in self.devices:
            if self.pozyx.getUWBSettings(settings, remote_id=remote_id):
                print(remote_id, settings)
                self.publish('settings', (remote_id, str(settings)))

    def set_settings(self):
        for remote_id in self.devices:
            settings = pypozyx.UWBSettings()
            settings.bitrate = 0  # {0: '110 kbit/s', 1: '850 kbit/s', 2: '6.81 Mbit/s'}
            settings.channel = 5
            settings.prf = 2  # {1: '16 MHz', 2: '64 MHz'}
            settings.gain_db = 11.5
            # {0x0C: '4096 symbols', 0x28: '2048 symbols', 0x18: '1536 symbols', 0x08: '1024 symbols',
            #  0x34: '512 symbols', 0x24: '256 symbols', 0x14: '128 symbols', 0x04: '64 symbols'}
            settings.plen = 0x08
            if self.pozyx.setUWBSettings(settings, remote_id=remote_id):
                print('set OK', remote_id)
            else:
                print('ERROR', remote_id)

    def run(self):
        try:
            self.get_settings()
            self.set_settings()
            self.get_settings()
            device_range = pypozyx.DeviceRange()
            while self.bus.is_alive():
                for from_id, to_id in itertools.combinations(self.devices, 2):
                    status = self.pozyx.doRanging(from_id, device_range, to_id)
                    if self.verbose:
                        print(device_range)
                    self.publish('range', [status, from_id, to_id, [device_range.timestamp, device_range.distance, device_range.RSS]])
                    if self.sleep_sec is not None:
                        self.sleep(self.sleep_sec)
        except BusShutdownException:
            pass


if __name__ == '__main__':
    import argparse
    import os.path
    import matplotlib.pyplot as plt
    from osgar.logger import lookup_stream_id, LogReader
    from osgar.lib.serialize import deserialize


    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('logfile', help='logfile path')
    parser.add_argument('-t', '--timestamps', help='use Pozyx timestamps', action='store_true')
    args = parser.parse_args()

    stream_id = lookup_stream_id(args.logfile, 'pozyx.range')
    arr = []
    for dt, channel, raw in LogReader(args.logfile, only_stream_id=stream_id):
        data = deserialize(raw)
        assert data[0] in [0, 1, 8], data
        if data[0] != pypozyx.POZYX_SUCCESS:
            continue
        dist = data[3][1]  # range
        if dist < 100000:  # 100m limit
            if args.timestamps:
                t = data[3][0]/1000.0  # timestamp (sec)
            else:
                t = dt.total_seconds()
            arr.append((t, (data[1], data[2]), dist))
        else:
            print(data)

    print(len(arr))

    groups = set([from_to for _, from_to, _ in arr])
    print(groups)

    for from_to in groups:
        tmp = [(t, dist) for t, link, dist in arr if link == from_to]
        x = [t for t, dist in tmp]
        y = [dist/1000.0 for t, dist in tmp]
        plt.plot(x, y, 'o', label=from_to)
    plt.xlabel('time (s)')
    plt.ylabel('range (m)')
    plt.legend()
    plt.title(os.path.basename(args.logfile))
    plt.show()

# vim: expandtab sw=4 ts=4
