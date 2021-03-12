"""
  OSGAR breadcrumbs dispenser for Virtual
"""
from ast import literal_eval
import struct
from collections import defaultdict
import math

import numpy as np
import cv2

from osgar.node import Node
from subt.trace import distance3D
from osgar.lib.pplanner import find_path

# http://www.arminhornung.de/Research/pub/hornung13auro.pdf
# 00: unknown; 01: occupied; 10: free; 11: inner node with child next in the stream

map2name = {
    0x0: 'unknown',
    0x1: 'occupied',
    0x2: 'free',
    0x3: 'node'
}


def Xdraw_node(data, img, size, offset_x=0, offset_y=0, pos=0):
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


def bin2code(d):
    ret = []
    for rot in range(0, 16, 2):
        val = (d & (0x3 << rot)) >> rot
        ret.append(map2name[val])
    print(ret)
    return ret


def draw_node(data, img, size, offset_x=0, offset_y=0, pos=0):
    img[:, :, :] = 0x80
    stack = []
    size = 100
    for i in range(10):
        d = struct.unpack_from('<H', data, pos)[0]
        for index, code in enumerate(bin2code(d)):
            if code == 'node':
                stack.append(pos)
            elif index < 4:
                x = index % 2
                y = index // 2
                w = size//2
                img[offset_x + x * w: offset_x + w + x * w, offset_y + y * w: offset_y + w + y * w, :] = 0x00
        pos += 1
        print(pos, len(stack))
        if len(stack) == 0:
            break
        stack = stack[1:]  # should be FIFO

#        size //= 2
        offset_x += size
        offset_y += size


def draw_map(data):
    img = np.zeros((1024, 1024, 3), dtype=np.uint8)
    draw_node(data, img, size=1024)
    return img


def dump_octomap(data):
    stack = [[]]
    for i in range(len(data) // 2):
        prefix = stack.pop(0)
        d = struct.unpack_from('<H', data, i * 2)[0]
        print(len(prefix), prefix, hex(d))
        for rot in range(14, -2, -2):
            val = (d & (0x3 << rot)) >> rot
            if val == 3:
                stack.insert(0, prefix + [rot // 2])
    assert 0, "dump end"


def seq2xyz(seq_arr):
    xyz = []
    max_len = max([len(s) for s in seq_arr])
    for seq in seq_arr:
        d = 2 ** (max_len - 1)
        x, y, z = -32767, -32767, -32767
        for code in seq:
            if code in [1, 3, 5, 7]:
                x += d
            if code in [2, 3, 6, 7]:
                y += d
            if code in [4, 5, 6, 7]:
                z += d
            d //= 2
        xyz.append(((x, y, z), len(seq)))
    return xyz


def xyz2img(img, xyz, color, level=2):
    for pos, size in xyz:
        x, y, z = pos
        assert 1 <= size <= 16, size
        d = 2 ** (16 - size)
        if z <= level < z + d:
            if d > 100:
                # do not try to fill extra large (unknown) squares, for now
                continue
            for dx in range(2*d - 1):
                for dy in range(2*d - 1):
                    px = 512 + 2*x + dx
                    py = 512 - 2*y - dy
                    if 0 <= px < 1024 and 0 <= py < 1024:
                        assert (img[py, px, 0], img[py, px, 1], img[py, px, 2]) == (0, 0, 0), (px, py, img[py, px, :], color, z, size)
                        img[py, px, 0] = color[0]
                        img[py, px, 1] = color[1]
                        img[py, px, 2] = color[2]
    return img


def data2stack(data):
    stack = [[]]
    unknown = []
    free = []
    occupied = []
    for i in range(len(data) // 2):
        prefix = stack.pop(0)
        d = struct.unpack_from('<H', data, i * 2)[0]
        for rot in range(14, -2, -2):  # range(0, 16, 2):
            val = (d & (0x3 << rot)) >> rot
            if val == 3:
                stack.insert(0, prefix + [rot // 2])
            elif val == 2:
                occupied.append(prefix + [rot // 2])
            elif val == 1:
                free.append(prefix + [rot // 2])
            elif val == 0:
                unknown.append(prefix + [rot // 2])
    assert len(stack) == 0, len(stack)
    return occupied, free, unknown


def data2maplevel(data, level):
    """
    Convert Octomap data to image/level
    """
    img = np.zeros((1024, 1024, 3), dtype=np.uint8)

    occupied, free, unknown = data2stack(data)
    xyz = seq2xyz(free)
    xyz2img(img, xyz, color=(0xFF, 0xFF, 0xFF), level=level)

    xyz = seq2xyz(occupied)
    xyz2img(img, xyz, color=(0x00, 0x00, 0xFF), level=level)

    xyz = seq2xyz(unknown)
    xyz2img(img, xyz, color=(0, 0xFF, 0), level=level)
    return img


class Octomap(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('waypoints')
        self.prev_data = None
        self.time_limit_sec = 60
        self.debug_arr = []
        self.waypoints = None  # experimental trigger of navigation
        self.start_xyz = None
        self.sim_time_sec = None
        self.pose3d = None

    def on_sim_time_sec(self, data):
        pass

    def on_pose3d(self, data):
        if self.start_xyz is None:
            self.start_xyz = data[0]
        x, y, z = data[0]
        if math.hypot(x - 3, y) < 2 and self.waypoints is not None:
            # in the tunnel
            print(data)
            self.publish('waypoints', self.waypoints)
            self.waypoints = None

    def on_octomap_obsolete(self, data):
        return  # do not worry about ocotomap for this test
        if self.time.total_seconds() < self.time_limit_sec:
            return
        assert len(data) % 2 == 0, len(data)
        data = bytes([(d + 256)%256 for d in data])

        occupied, free, unknown = data2stack(data)

        xyz = seq2xyz(occupied)
        self.debug_arr.append(xyz)
        self.time_limit_sec += 60

        if self.time.total_seconds() < 7 * 60:
            return

        for level in range(-10, 10):
            img = np.zeros((1024, 1024, 3), dtype=np.uint8)

            xyz = seq2xyz(free)
            xyz2img(img, xyz, color=(0xFF, 0xFF, 0xFF), level=level)

            xyz = seq2xyz(occupied)
            xyz2img(img, xyz, color=(0x00, 0x00, 0xFF), level=level)

            xyz = seq2xyz(unknown)
            xyz2img(img, xyz, color=(0, 0xFF, 0), level=level)
            cv2.imwrite('octo_%03d.png' % (level + 10), img)
        assert 0, "END jpg"


        import matplotlib as mpl
        import matplotlib.pyplot as plt

        self.debug_arr.reverse()
        for xyz in self.debug_arr:
            x = [a for a, _, _ in xyz]
            y = [a for _, a, _ in xyz]
            line = plt.plot(x, y, 'o')
        plt.axes().set_aspect('equal', 'datalim')

        plt.show()
        assert 0, "END"

    def on_octomap(self, data):
#        if self.sim_time_sec is None or self.sim_time_sec < 10: #self.time_limit_sec:
        if self.pose3d is None or self.time.total_seconds() < self.time_limit_sec:
            return
        self.time_limit_sec += 60

        assert len(data) % 2 == 0, len(data)
        data = bytes([(d + 256) % 256 for d in data])

        x = self.pose3d[0][0] - self.start_xyz[0]
        y = self.pose3d[0][1] - self.start_xyz[1]
        start = int(512 + 4*x), int(512 - 4*y)
        img = data2maplevel(data, level=1)  # 0.5m above the ground?
        img2, path = frontiers(img, start)
        cv2.circle(img2, start, radius=2, color=(39, 127, 255), thickness=-1)
        cv2.imwrite('octo_cut.png', img2)

        if path is not None:
            self.waypoints = [((x - 512)/4 + self.start_xyz[0], (512 - y)/4 + self.start_xyz[1], 0) for x, y in path]

    def update(self):
        channel = super().update()
        handler = getattr(self, "on_" + channel, None)
        if handler is not None:
            handler(getattr(self, channel))
        else:
            assert False, channel  # unknown channel


###############################################################################
def frontiers(img, start, draw=False):
    green = (img[:, :, 0] == 0) & (img[:, :, 1] == 255) & (img[:, :, 2] == 0)
    white = (img[:, :, 0] == 255) & (img[:, :, 1] == 255) & (img[:, :, 2] == 255)

    mask_right = green[:, 2:] & white[:, :-2]
    mask_left = green[:, :-2] & white[:, 2:]
    mask = mask_left | mask_right
    z = np.zeros((1024, 1), dtype=np.bool)
    mask = np.hstack([z, mask, z])

    mask_up = green[2:, :] & white[:-2, :]
    mask_down = green[:-2, :] & white[2:, :]
    z = np.zeros((1, 1024), dtype=np.bool)
    mask2 = mask_up | mask_down
    mask = np.vstack([z, mask2, z]) | mask

    xy = np.where(mask)
    score = np.zeros(len(xy[0]))
    for i in range(len(xy[0])):
        x, y = xy[0][i]-512, 512-xy[1][i]
        score[i] = math.hypot(x, y) * 0.03
        for j in range(len(xy[0])):
            x2, y2 = xy[0][j]-512, 512-xy[1][j]
            dist = math.hypot(x - x2, y - y2)
            if dist < 10:  # ~ 5 meters
                score[i] += 1.0

    if draw:
        import matplotlib.pyplot as plt
        line = plt.plot(xy[1]-512, 512-xy[0], 'bo')
        m = score > 3*max(score)/4
        plt.plot(xy[1][m] - 512, 512 - xy[0][m], 'ro')

        plt.axes().set_aspect('equal', 'datalim')
        plt.show()

    driveable = white[:1023, :1023] | white[1:, :1023] | white[1:, 1:] | white[:1023, 1:]
    i = np.argmax(score)
    limit_score = 3*max(score)/4
    goals = [(xy[1][i], xy[0][i]) for i in range(len(xy[0])) if score[i] > limit_score]
    path = find_path(driveable, start, goals, verbose=False)

    img[mask, 0] = 255  # pink
    img[mask, 1] = 0
    img[mask, 2] = 255

    if path is not None:
        for x, y in path:
            img[y][x][0] = 255
            img[y][x][1] = 0
            img[y][x][2] = 0
    else:
        print('Path not found!')

    return img, path


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser("Analyze ocotomap slice")
    parser.add_argument('imgpath', help='path to PNG slice image')
    parser.add_argument('--out', help='output path to PNG image', default='out.png')
    parser.add_argument('--draw', action='store_true', help='draw pyplot frontiers')
    args = parser.parse_args()

    img = cv2.imread(args.imgpath, 1)
    img2 = frontiers(img, args.draw)
    cv2.imwrite(args.out, img2)


# vim: expandtab sw=4 ts=4
