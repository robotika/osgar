"""
  OSGAR Pozyx (utrawide-band trilateration) wrapper
"""
import itertools
import math
from collections import defaultdict

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
            settings.channel = 1
            settings.prf = 2  # {1: '16 MHz', 2: '64 MHz'}
            settings.gain_db = 33.0
            # {0x0C: '4096 symbols', 0x28: '2048 symbols', 0x18: '1536 symbols', 0x08: '1024 symbols',
            #  0x34: '512 symbols', 0x24: '256 symbols', 0x14: '128 symbols', 0x04: '64 symbols'}
            settings.plen = 0x0C
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

############### supporting tools #####################

def read_data(logfile):
    from osgar.logger import lookup_stream_id, LogReader
    from osgar.lib.serialize import deserialize

    stream_id = lookup_stream_id(logfile, 'pozyx.range')
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
    return arr


def draw_ranges(arr, title):
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
    plt.title(title)
    plt.show()


def create_map(arr, anchors, selected=None):
    ret = []
    x, y = 0, 0
    for t, (src_node, dst_node), dist_mm in arr:
        dist = dist_mm/1000.0
        if src_node in anchors and dst_node in anchors:
            a = anchors[src_node]
            b = anchors[dst_node]
            map_dist = math.hypot(a[0] - b[0], a[1] - b[1])
            if abs(map_dist - dist) > 1.0:
                print(a, b, map_dist, dist)
        else:
            if src_node in anchors:
                xyz = anchors[src_node]
                tag = dst_node
            elif dst_node in anchors:
                xyz = anchors[dst_node]
                tag = src_node
            else:
#                assert 0, f'Incomplete anchors! {anchors}, {src_node, dst_node}'
                continue
            if tag != selected:
                continue
#            assert tag is None, tag
            vec = xyz[0] - x, xyz[1] - y
            vec_size = math.hypot(vec[0], vec[1])
            if abs(vec_size) < 0.1:
                continue
            err = vec_size - dist
            k = err
            x += k * vec[0]/vec_size
            y += k * vec[1]/vec_size
            ret.append((x, y))
    return ret


def draw_map(m, title):
    arr_x = [x for x, y in m]
    arr_y = [y for x, y in m]
    plt.plot(arr_x, arr_y, 'o-')
    plt.axes().set_aspect('equal', 'datalim')
    plt.title(title)


def get_anchors_xyz(arr, anchor_names):
    import numpy as np

    dist_array = defaultdict(list)
    for t, (src_node, dst_node), dist_mm in arr:
        dist = dist_mm/1000.0
        if src_node in anchor_names and dst_node in anchor_names:
            dist_array[(src_node, dst_node)].append(dist)
    res = dict([(pair, np.median(d)) for pair, d in dist_array.items()] +
               [((pair[1], pair[0]), np.median(d)) for pair, d in dist_array.items()])

    assert len(anchor_names) >= 3, anchor_names  # at least triangle
    A, B, C = anchor_names[:3]
    ret = {}
    a, b, c = res[(B, C)], res[(A, C)], res[(A, B)]
    ret[A] = (0, 0, 0)
    ret[B] = (c, 0, 0)
    angle = math.acos((a*a - c*c - b*b)/(-2*c*b))
    ret[C] = (b*math.cos(angle), b*math.sin(angle), 0)
    return ret


if __name__ == '__main__':
    import argparse
    import os.path
    import matplotlib.pyplot as plt

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('logfile', help='logfile path')
    parser.add_argument('-t', '--timestamps', help='use Pozyx timestamps', action='store_true')
    parser.add_argument('--map', help='display map instead of ranges', action='store_true')
    args = parser.parse_args()

    title = os.path.basename(args.logfile)
    arr = read_data(args.logfile)
    print(len(arr))
    if args.map:
        old_anchors = {
            0x0D67: (0, 0, 0),
            0x0D7F: (10.4, 0, 0),
            0x0D53: (12.721, 29.308, 0),
            0x6826: (5, 30, 0)
        }
        anchors = get_anchors_xyz(arr, [3431, 26727, None])
        m = create_map(arr, anchors, selected=0xD7F)  # 3455
        draw_map(m, title)
        m = create_map(arr, anchors, selected=3411)
        draw_map(m, title)
        plt.show()
    else:
       draw_ranges(arr, title)

# vim: expandtab sw=4 ts=4
