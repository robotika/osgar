"""
  OSGAR breadcrumbs dispenser for Virtual
"""
from ast import literal_eval
import struct
from collections import defaultdict

import numpy as np

from osgar.node import Node
from subt.trace import distance3D

# http://www.arminhornung.de/Research/pub/hornung13auro.pdf
# 00: unknown; 01: occupied; 10: free; 11: inner node with child next in the stream

def draw_node(data, img, size, offset_x=0, offset_y=0, pos=0):
    arr = [(size, offset_x, offset_y, pos)]

    img[:, :, :] = 0x80

    pos = 0
    while len(arr) > 0:  # and pos < 10000:
        size, offset_x, offset_y, pos_read = arr[0]
        arr = arr[1:]
#        print(pos_read, pos, size, offset_x, offset_y)
        d = struct.unpack_from('<H', data, pos_read)[0]
        w = size//2
        for i in range(8):
            value = (d >> (2*i)) & 0x3
            x, y, z = i & 0x1, (i & 0x2) >> 1, (i & 0x4) >> 2
            x, y = y, z
            color_tab = [0x80, 0x00, 0xFF, 0x33]
            if value == 3:
                pos += 2
                arr.append((w, offset_x+x*w, offset_y+y*w, pos))
            elif value in [1, 2]:
                img[offset_x + x*w : offset_x + w+x*w, offset_y + y*w : offset_y + w+y*w, :] = color_tab[value]


def draw_map(data):
    img = np.zeros((1024, 1024, 3), dtype=np.uint8)
    draw_node(data, img, size=1024)
    return img


class Octomap(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register("size")
        self.prev_data = None

    def on_sim_time_sec(self, data):
        pass

    def on_pose3d(self, data):
        pass

    def on_octomap(self, data):
        assert len(data) % 2 == 0, len(data)
#        print(len(data))
        data = bytes([(d + 256)%256 for d in data])
#        print(data[:2])
#        with open('freyja-octomap.bin', 'wb') as f:
#            f.write(data)
        i = 0
        d = struct.unpack_from('<H', data, i)[0]
#        print(len(data)) # hex(d))
        if self.prev_data is not None and len(self.prev_data) + 22 == len(data):
            print('diff', len(data) - len(self.prev_data))
            print(data[:10])
            print(self.prev_data[:10])
#            with open('freyja-octomap-prev.bin', 'wb') as f:
#                f.write(self.prev_data)
#            assert 0, 'END'

        stat = defaultdict(int)
        for i in range(len(data)//2):
            d = struct.unpack_from('<H', data, i * 2)[0]
            for rot in range(0, 16, 2):
                val = (d & (0x3<<rot))>>rot
                if val == 3:
                    stat[rot] += 1
#                stat[(d & (0x3<<rot))>>rot] += 1
        print(sorted(stat.items()))
        self.prev_data = data[:]

    def update(self):
        channel = super().update()
        handler = getattr(self, "on_" + channel, None)
        if handler is not None:
            handler(getattr(self, channel))

# vim: expandtab sw=4 ts=4
