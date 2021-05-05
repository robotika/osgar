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

STATE_UNKNOWN = 128   # color=(0, 0xFF, 0)
STATE_FREE = 255      # color=(0xFF, 0xFF, 0xFF)
STATE_OCCUPIED = 1    # color=(0x00, 0x00, 0xFF)  ... just to be != 0, which is original unknown/black/undefined
STATE_FRONTIER = 196  # color=(0xFF, 0x00, 0xFF)
STATE_PATH     = 64   # color=(0xFF, 0x00, 0x00)

SLICE_OCTOMAP_SIZE = 1024  # size of slice/image in XY octomap coordinates (for given Z)


def seq2xyz(seq_arr):
    """
    Convert octomap sequence (0..7) into XYZ coordinate
    :param seq_arr: list of parent-child sequences for one of given type (free, occupied, unknown)
    :return: list of XYZ boxes with their "size category" (shorted the sequence bigger the voxel)
    """
    xyz = []
    if len(seq_arr) == 0:
        return xyz
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
    :param color: value 0..255 to be assigned to voxels at given level
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
            px = SLICE_OCTOMAP_SIZE//2 + x
            py = SLICE_OCTOMAP_SIZE//2 - y
            img[max(0, py-d+1):min(SLICE_OCTOMAP_SIZE, py+1), max(0, px):min(SLICE_OCTOMAP_SIZE, px+d)] = color
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
    img = np.zeros((SLICE_OCTOMAP_SIZE, SLICE_OCTOMAP_SIZE), dtype=np.uint8)

    occupied, free, unknown = data2stack(data)
    xyz = seq2xyz(free)
    xyz2img(img, xyz, color=STATE_FREE, level=level)

    xyz = seq2xyz(occupied)
    xyz2img(img, xyz, color=STATE_OCCUPIED, level=level)

    xyz = seq2xyz(unknown)
    xyz2img(img, xyz, color=STATE_UNKNOWN, level=level)
    return img


def frontiers(img, start, draw=False):
    """
    Find path to the best frontier (free-unknown transition)
    :param img: color image with free=white, unknown=green
    :param start: start pixel
    :param draw: debug frontiers in pyplot
    :return: extended image with drawn start and path, path
    """
    size = img.shape
    green = img[:, :, :] == STATE_UNKNOWN
    white = img[:, :, :] == STATE_FREE

    mask_right = green[:, 2:, :] & white[:, 1:-1, :]
    mask_left = green[:, :-2, :] & white[:, 1:-1, :]
    mask = mask_left | mask_right
    z = np.zeros((size[0], 1, size[2]), dtype=np.bool)
    mask = np.hstack([z, mask, z])

    mask_up = green[2:, :, :] & white[1:-1, :, :]
    mask_down = green[:-2, :, :] & white[1:-1, :, :]
    z = np.zeros((1, size[1], size[2]), dtype=np.bool)
    mask2 = mask_up | mask_down
    mask = np.vstack([z, mask2, z]) | mask

    z_mask_up = green[:, :, 2:] & white[:, :, 1:-1]
    z_mask_down = green[:, :, :-2] & white[:, :, 1:-1]
    z = np.zeros((size[0], size[1], 1), dtype=np.bool)
    mask3 = z_mask_up | z_mask_down
#    mask = np.concatenate([z, mask3, z], axis=2) | mask

    xy = np.where(mask)
    if len(xy[0]) == 0:
        # there are no frontiers, i.e. no exploration path
        return img, None

    score = np.zeros(len(xy[0]))
    for i in range(len(xy[0])):
        x, y = xy[0][i]-SLICE_OCTOMAP_SIZE//2, SLICE_OCTOMAP_SIZE//2-xy[1][i]
        score[i] = math.hypot(x, y) * 0.03
        for j in range(len(xy[0])):
            x2, y2 = xy[0][j]-SLICE_OCTOMAP_SIZE//2, SLICE_OCTOMAP_SIZE//2-xy[1][j]
            dist = math.hypot(x - x2, y - y2)
            if dist < 10:  # ~ 5 meters
                score[i] += 1.0

    if draw:
        import matplotlib.pyplot as plt
        line = plt.plot(xy[1]-SLICE_OCTOMAP_SIZE//2, SLICE_OCTOMAP_SIZE//2-xy[0], 'bo')
        m = score > 3*max(score)/4
        plt.plot(xy[1][m] - SLICE_OCTOMAP_SIZE//2, SLICE_OCTOMAP_SIZE//2 - xy[0][m], 'ro')

        plt.axes().set_aspect('equal', 'datalim')
        plt.show()

    drivable = white

    # use 1 pixel surrounding
    drivable_safe_y = drivable[2:, :] & drivable[1:-1, :] & drivable[:-2, :]
    drivable_safe_xy = drivable_safe_y[:, 2:] & drivable_safe_y[:, 1:-1] & drivable_safe_y[:, :-2]
    # add non-drivable frame to match original image size
    z = np.zeros((size[0] - 2, 1, size[2]), dtype=np.bool)
    tmp = np.hstack([z, drivable_safe_xy, z])
    z = np.zeros((1, size[1], size[2]), dtype=np.bool)
    drivable = np.vstack([z, tmp, z])

    limit_score = 3*max(score)/4
    # select goal positions above the limit_score
    # note, that the "safe path" does not touch external boundary so it would never find path
    # to frontier. As a workaround add also all 8-neighbors of frontiers.
    goals = []
    xy = np.array(xy)[:, score > limit_score]
    for dx in [-1, 0, 1]:
        for dy in [-1, 0, 1]:
            for dz in [0]:  #[-1, 0, 1]:
                goals.append(xy + np.repeat(np.asarray([[dy], [dx], [dz]]), xy.shape[1], axis=1))
    goals = np.hstack(goals).T[:, [1, 0, 2]]

    # the path planner currently expects goals as tuple (x, y) and operation "in"
    goals = set(map(tuple, goals))
    path = find_path(drivable, start, goals, verbose=False)

    img[mask] = STATE_FRONTIER

    if path is not None:
        for x, y, z in path:
            img[y][x] = STATE_PATH
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
        self.sim_time_sec = None
        self.pose3d = None
        self.video_writer = None
        self.video_outfile = None  # 'octo.mp4'  # optional video output generation
        self.min_z = config.get('min_z', 0.5)  # should be multiply of "resolution"
        self.max_z = config.get('max_z', 0.5)  # the limits are included
        self.resolution = config.get('resolution', 0.5)
        self.verbose = False

    def on_sim_time_sec(self, data):
        if self.time_limit_sec is None:
            self.time_limit_sec = data

    def on_pose3d(self, data):
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

        x, y, z = self.pose3d[0]
        start = int(SLICE_OCTOMAP_SIZE//2 + x/self.resolution), int(SLICE_OCTOMAP_SIZE//2 - y/self.resolution), int((z - self.min_z)/self.resolution)
        num_z_levels = int(round((self.max_z - self.min_z)/self.resolution)) + 1
        img3d = np.zeros((SLICE_OCTOMAP_SIZE, SLICE_OCTOMAP_SIZE, num_z_levels), dtype=np.uint8)
        for level in range(num_z_levels):
            img3d[:, :, level] = data2maplevel(data, level=level + int(round(self.min_z/self.resolution)))

        if self.verbose:
            for i in range(num_z_levels):
                cv2.imwrite('octo_%03d.png' % i, img3d[:, :, i])

        img2 = np.zeros((SLICE_OCTOMAP_SIZE, SLICE_OCTOMAP_SIZE, 3), dtype=np.uint8)
        level = max(0, min(num_z_levels - 1, start[2]))
        img2[:, :, 0] = img3d[:, :, level]
        img2[:, :, 1] = img3d[:, :, level]
        img2[:, :, 2] = img3d[:, :, level]
        __, path = frontiers(img3d, start)  # this image is modified in place anyway
        if self.verbose:
            f = (img3d == STATE_FRONTIER).nonzero()
            for x, y, z in zip(f[1], f[0], f[2]):
                if z == start[2]:
                    cv2.circle(img2, (x, y), radius=0, color=(255, 0, 255), thickness=-1)
            if path is not None:
                for x, y, z in path:
                    cv2.circle(img2, (x, y), radius=0, color=(255, 0, 0), thickness=-1)

            cv2.circle(img2, start[:2], radius=0, color=(39, 127, 255), thickness=-1)
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
            self.waypoints = [[(x - SLICE_OCTOMAP_SIZE//2)/2,
                               (SLICE_OCTOMAP_SIZE//2 - y)/2,
                               z * self.resolution + self.min_z]
                              for x, y, z in path]

    def update(self):
        channel = super().update()
        handler = getattr(self, "on_" + channel, None)
        if handler is not None:
            handler(getattr(self, channel))
        else:
            assert False, channel  # unknown channel


if __name__ == "__main__":
    import argparse
    from osgar.lib.serialize import deserialize
    from osgar.logger import LogReader, lookup_stream_id

    parser = argparse.ArgumentParser("Analyze ocotomap data")
    parser.add_argument('logfile', help='path to logfile with octomap data')
    parser.add_argument('--out', help='output path to PNG image', default='out.png')
    parser.add_argument('--draw', action='store_true', help='draw pyplot frontiers')
    args = parser.parse_args()

    octomap_stream_id = lookup_stream_id(args.logfile, 'fromrospy.octomap')
    pose3d_stream_id = lookup_stream_id(args.logfile, 'fromrospy.pose3d')
    waypoints_stream_id = lookup_stream_id(args.logfile, 'octomap.waypoints')
    pose3d = None
    x, y, z = 0, 0, 0
    resolution = 0.5
    waypoints = None
    with LogReader(args.logfile,
               only_stream_id=[octomap_stream_id, pose3d_stream_id, waypoints_stream_id]) as logreader:
        level = 2
        for time, stream, data in logreader:
            data = deserialize(data)
            if stream == pose3d_stream_id:
                pose3d = data
                x, y, z = pose3d[0]
                start = int(SLICE_OCTOMAP_SIZE//2 + x/resolution), int(SLICE_OCTOMAP_SIZE//2 - y/resolution), int(z / resolution)
                continue

            if stream == waypoints_stream_id:
                waypoints = data
                continue

            if waypoints is None:
                # speed up display/processing - maybe optional?
                continue

            assert len(data) % 2 == 0, len(data)  # TODO fix this in cloudsim2osgar
            data = bytes([(d + 256) % 256 for d in data])

            paused = False
            while True:
                img = data2maplevel(data, level=level)
                cv2.circle(img, start[:2], radius=0, color=(39, 127, 255), thickness=-1)
                img = cv2.resize(img[256+128:-256-128, 256+128:-256-128], img.shape)
                cv2.imshow('Octomap', img)
                pose_str = '(%.02f, %.02f, %.02f)' % tuple(pose3d[0]) if pose3d is not None else 'None'
                cv2.setWindowTitle('Octomap', f'Octomap {time}, {pose_str}, level={level}' + (' (paused)' if paused else ''))
                key = cv2.waitKey(1) & 0xFF
                KEY_Q = ord('q')
                if key == KEY_Q:
                    break
                if key == ord(' '):
                    paused = not paused
                if ord('0') <= key <= ord('9'):
                    level = key - ord('0')
                if key == ord('d'):
                    import open3d as o3d
                    all = []
                    for lev in range(-10, 20):
                        img = data2maplevel(data, level=lev)
                        xy = np.where(img == STATE_OCCUPIED)
                        xyz = np.array([xy[0], xy[1], np.full(len(xy[0]), lev)]).T
                        all.extend(xyz.tolist())
                    pcd = o3d.geometry.PointCloud()
                    xyz = np.array(all)
                    pcd.points = o3d.utility.Vector3dVector(xyz)
                    voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, voxel_size=0.5)
                    colors = [[1, 0, 0] for i in range(len(xyz))]

                    print(waypoints)
                    res = 1  #0.5
                    points = [[SLICE_OCTOMAP_SIZE/2 + x/res, SLICE_OCTOMAP_SIZE + y/res, z/res] for x, y, z in waypoints]
                    lines = [[i, i+1] for i in range(len(points) - 1)]
                    colors = [[1, 0, 0] for i in range(len(lines))]
                    line_set = o3d.geometry.LineSet()
                    line_set.points = o3d.utility.Vector3dVector(points)
                    line_set.lines = o3d.utility.Vector2iVector(lines)
                    line_set.colors = o3d.utility.Vector3dVector(colors)
                    o3d.visualization.draw_geometries([voxel_grid, line_set])
                if not paused:
                    break
            if key == KEY_Q:
                break
            waypoints = None  # force wait for next waypoints message

# vim: expandtab sw=4 ts=4
