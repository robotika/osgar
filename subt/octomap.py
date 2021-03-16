"""
  ROS Binary Octomap parsing and processing
"""
import struct
import math

import numpy as np
import cv2

from osgar.node import Node
from osgar.lib.pplanner import find_path

# http://www.arminhornung.de/Research/pub/hornung13auro.pdf
# 00: unknown; 01: occupied; 10: free; 11: inner node with child next in the stream


def seq2xyz(seq_arr):
    """
    Convert octomap sequence (0..7) into XYZ coordinate
    :param seq_arr: list of parent-child sequences for one of given type (free, occupied, unknown)
    :return: list of XYZ boxes with their "size category" (shorted the sequence bigger the voxel)
    """
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
    """
    Draw given list of voxels into existing image
    :param img: I/O image
    :param xyz: list of voxels (xyz and "size")
    :param color: to be used for drawing
    :param level: Z-level for the cut
    :return: updated image
    """
    for pos, size in xyz:
        x, y, z = pos
        assert 1 <= size <= 16, size
        d = 2 ** (16 - size)
        if z <= level < z + d:
            if d > 100:
                # do not try to fill extra large (unknown) squares, for now
                continue
            for dx in range(d):
                for dy in range(d):
                    px = 512 + x + dx
                    py = 512 - y - dy
                    if 0 <= px < 1024 and 0 <= py < 1024:
                        assert (img[py, px, 0], img[py, px, 1], img[py, px, 2]) == (0, 0, 0), (px, py, img[py, px, :], color, z, size)
                        img[py, px, 0] = color[0]
                        img[py, px, 1] = color[1]
                        img[py, px, 2] = color[2]
    return img


def data2stack(data):
    """
    Convert binary ocotomap data into three lists (occupied, free, unknown)
    :param data: binary octomap data (depth first)
    :return: (occupied, free, unknown) lists of sequences
    """
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


def frontiers(img, start, draw=False):
    """
    Find path to the best frontier (free-unknown transition)
    :param img: color image with free=white, unknown=green
    :param start: start pixel
    :param draw: debug frontiers in pyplot
    :return: extended image with drawn start and path, path
    """
    green = (img[:, :, 0] == 0) & (img[:, :, 1] == 255) & (img[:, :, 2] == 0)
    white = (img[:, :, 0] == 255) & (img[:, :, 1] == 255) & (img[:, :, 2] == 255)

    mask_right = green[:, 2:] & white[:, 1:-1]
    mask_left = green[:, :-2] & white[:, 1:-1]
    mask = mask_left | mask_right
    z = np.zeros((1024, 1), dtype=np.bool)
    mask = np.hstack([z, mask, z])

    mask_up = green[2:, :] & white[1:-1, :]
    mask_down = green[:-2, :] & white[1:-1, :]
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

    drivable = white

    # use 1 pixel surrounding
    drivable_safe_y = drivable[2:, :] & drivable[1:-1, :] & drivable[:-2, :]
    drivable_safe_xy = drivable_safe_y[:, 2:] & drivable_safe_y[:, 1:-1] & drivable_safe_y[:, :-2]
    # add non-drivable frame to match original image size
    z = np.zeros((1022, 1), dtype=np.bool)
    tmp = np.hstack([z, drivable_safe_xy, z])
    z = np.zeros((1, 1024), dtype=np.bool)
    drivable = np.vstack([z, tmp, z])

    img[drivable, : ] = 128  # gray

    i = np.argmax(score)
    limit_score = 3*max(score)/4
    # select goal positions above the limit_score
    # note, that the "safe path" does not touch external boundary so it would never find path
    # to frontier. As a workaround add also all 8-neighbors of frontiers.
    goals = []
    xy = np.array(xy)[:, score > limit_score]
    for dx in [-1, 0, 1]:
        for dy in [-1, 0, 1]:
            goals.append(xy + np.repeat(np.asarray([[dy], [dx]]), xy.shape[1], axis=1))
    goals = np.hstack(goals).T[:, ::-1]

    # the path planner currently expects goals as tuple (x, y) and operation "in"
    goals = set(map(tuple, goals))
    path = find_path(drivable, start, goals, verbose=False)

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


class Octomap(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('waypoints')
        self.prev_data = None
        self.time_limit_sec = None  # initialized with the first sim_time_sec
        self.debug_arr = []
        self.waypoints = None  # experimental trigger of navigation
        self.start_xyz = None
        self.sim_time_sec = None
        self.pose3d = None
        self.video_writer = None
        self.video_outfile = None  # 'octo.mp4'  # optional video output generation
        self.zlevel = config.get('zlevel', 0.5)

    def on_sim_time_sec(self, data):
        if self.time_limit_sec is None:
            self.time_limit_sec = data

    def on_pose3d(self, data):
        if self.start_xyz is None:
            # the octomap starts with robot position at (0, 0, 0) - correction offset is needed
            self.start_xyz = data[0]
        if self.waypoints is not None:
            print('Waypoints', data[0], self.waypoints[0], self.waypoints[-1])
            self.publish('waypoints', self.waypoints)
            self.waypoints = None

    def on_octomap(self, data):
        if self.sim_time_sec is None or self.pose3d is None or self.sim_time_sec < self.time_limit_sec:
            return
        self.time_limit_sec += 5  # simulated seconds

        # bit unlucky conversion from existing Python2 data
        assert len(data) % 2 == 0, len(data)
        data = bytes([(d + 256) % 256 for d in data])

        x = self.pose3d[0][0] - self.start_xyz[0]
        y = self.pose3d[0][1] - self.start_xyz[1]
        start = int(512 + 2*x), int(512 - 2*y)
        img = data2maplevel(data, level=int(round(self.zlevel/0.5)))  # 0.5m above the ground?
        img2, path = frontiers(img, start)
        cv2.circle(img2, start, radius=0, color=(39, 127, 255), thickness=-1)
        cv2.imwrite('octo_cut.png', img2)  # used for replay debugging

        if self.video_outfile is not None:
            if self.video_writer is None:
                fps = 1
                height, width = img2.shape[:2]
                self.video_writer = cv2.VideoWriter(self.video_outfile,
                                         cv2.VideoWriter_fourcc(*"mp4v"),
                                         fps,
                                         (width, height))
            self.video_writer.write(img2)

        if path is not None:
            self.waypoints = [[(x - 512)/2 + self.start_xyz[0], (512 - y)/2 + self.start_xyz[1], self.zlevel] for x, y in path]

    def update(self):
        channel = super().update()
        handler = getattr(self, "on_" + channel, None)
        if handler is not None:
            handler(getattr(self, channel))
        else:
            assert False, channel  # unknown channel


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
