import math

import numpy as np
import matplotlib.pyplot as plt

from osgar.logger import LogReader, lookup_stream_id
from osgar.lib.serialize import deserialize



def scan_gen(filename):
    stream_id = lookup_stream_id(filename, 'vanjee.scan')
    with LogReader(filename, only_stream_id=stream_id) as log:
        for timestamp, stream_id, raw_data in log:
            data = deserialize(raw_data)
            assert len(data) == 1800, len(data)
            scan = []
            for i, d in enumerate(data):
                if d > 0:
                    a = math.radians(360*i/1800.0)
                    d /= 1000.0
                    x, y = d * math.cos(a), d * math.sin(a)
                    scan.append((x, y))
            yield scan


def nearest(scan1, scan2):
    """
    return pairs of nearest points from scan2 for each point in scan1
    """
    pairs = []
    for pt1 in scan1:
        min_d = None
        candidate = None
        for pt2 in scan2:
            d = math.hypot(pt1[0]-pt2[0], pt1[1]-pt2[1])
            if min_d is None or d < min_d:
                min_d = d
                candidate = pt2
        assert candidate is not None
        pairs.append((pt1, candidate))
    return pairs


def transform(pairs):
    """
    return translation and rotation matrix for given list of pair 2D points
    """
    sum_x1, sum_y1 = 0, 0
    sum_x2, sum_y2 = 0, 0
    for (x1, y1), (x2, y2) in pairs:
        sum_x1 += x1
        sum_y1 += y1
        sum_x2 += x2
        sum_y2 += y2

    xc1, yc1 = sum_x1/len(pairs), sum_y1/len(pairs)
    xc2, yc2 = sum_x2/len(pairs), sum_y2/len(pairs)

    a, b, c, d = 0, 0, 0, 0
    for (x1, y1), (x2, y2) in pairs:
        a += (x1-xc1) * (x2-xc2)
        b += (x1-xc1) * (y2-yc2)
        c += (y1-yc1) * (x2-xc2)
        d += (y1-yc1) * (y2-yc2)

    m = np.array([a, b, c, d]).reshape(2, 2)
    U, S, V = np.linalg.svd(m)
    rot = np.matmul(U, V)
    trans = np.array([xc1, yc1]) - np.matmul(rot, np.array([xc2, yc2]).T)
    return trans, rot


def draw_scans(scan1, scan2, pairs=None, filename=None):
    plt.clf()
    plt.plot(*zip(*scan1), '-x')
    plt.plot(*zip(*scan2), '-o')
    if pairs is not None:
        for (x1, y1), (x2, y2) in pairs:
            plt.plot([x1, x2], [y1, y2], 'g')
    plt.axis('equal')
#    plt.show()
    if filename is not None:
        plt.savefig(filename)


def my_icp(scan1, scan2, debug_path_prefix=None, num_iter=10, offset=0):
    for i in range(offset, offset + num_iter):
        pairs = nearest(scan1, scan2)
        filename = f'{debug_path_prefix}image{i:02}.png' if debug_path_prefix is not None else None
        if filename:
            print(filename)
        draw_scans(scan1, scan2, pairs, filename=filename)
        trans, rot = transform(pairs)
        scan2b = np.matmul(np.array(scan2), rot.T) + trans.T
        scan2 = scan2b

    draw_scans(scan1, scan2)
    return scan2


def run_icp(filename, debug_path_prefix=None, num_iter=10):
    prev = None
    offset = 0
    for i, scan in enumerate(scan_gen(filename)):
        if i % 10 != 0:
            continue
        if prev is not None:
            matched_scan = my_icp(prev, scan, debug_path_prefix=debug_path_prefix,
                                  num_iter=num_iter, offset=offset)
            offset += num_iter
            print(i, len(prev))
        prev = scan


def create_animation(path_prefix, num_iter=10):
    from PIL import Image
    frames = []
    for i in range(num_iter):
        filename = f'{path_prefix}image{i:02}.png'
        frames.append(Image.open(filename))
    assert len(frames) > 0
    frames[0].save(path_prefix + 'image.gif', format='GIF', append_images=frames,
                   save_all=True, duration=1000, loop=0)  # display in ms, number of loops


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('logfile', help='OSGAR logfile')
    parser.add_argument('--debug-path-prefix', '-d', help='save intermediate files')
    parser.add_argument('--num', '-n', help='number of iterations', type=int, default=10)
    args = parser.parse_args()
    run_icp(args.logfile, debug_path_prefix=args.debug_path_prefix, num_iter=args.num)
    if args.debug_path_prefix is not None:
        create_animation(args.debug_path_prefix, num_iter=args.num)
