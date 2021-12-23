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
        bus.register('range')
        serial_port = config['port']
        self.devices = [int(x, 16) for x in config.get('devices', [])]  # unfortunately JSON does not support hex
        self.devices.append(None)  # extra range to the base (must be last, 2nd param)
        self.pozyx = pypozyx.PozyxSerial(serial_port)
        self.verbose = False

    def run(self):
        try:
            device_range = pypozyx.DeviceRange()
            while self.bus.is_alive():
                for from_id, to_id in itertools.combinations(self.devices, 2):
                    status = self.pozyx.doRanging(from_id, device_range, to_id)
                    if self.verbose:
                        print(device_range)
                    self.publish('range', [status, from_id, to_id, [device_range.timestamp, device_range.distance, device_range.RSS]])
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
            t = dt.total_seconds()
#            t = data[3][0]/1000.0  # timestamp (sec)
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
