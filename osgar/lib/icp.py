import math

from simpleicp import PointCloud, SimpleICP
from simpleicp.simpleicp import SimpleICPException
import numpy as np
import matplotlib.pyplot as plt

from osgar.logger import LogReader, lookup_stream_id
from osgar.lib.serialize import deserialize


def demo():
    # Read point clouds from xyz files into n-by-3 numpy arrays
    X_fix = np.genfromtxt("bunny_part1.xyz")
    X_mov = np.genfromtxt("bunny_part2.xyz")

    # Create point cloud objects
    pc_fix = PointCloud(X_fix, columns=["x", "y", "z"])
    pc_mov = PointCloud(X_mov, columns=["x", "y", "z"])

    # Create simpleICP object, add point clouds, and run algorithm!
    icp = SimpleICP()
    icp.add_point_clouds(pc_fix, pc_mov)
    H, X_mov_transformed, rigid_body_transformation_params, distance_residuals = icp.run(max_overlap_distance=1)


def scan_gen(filename):
    stream_id = lookup_stream_id(filename, 'vanjee.scan')
    hack = 0
    with LogReader(filename, only_stream_id=stream_id) as log:
        for timestamp, stream_id, raw_data in log:
            data = deserialize(raw_data)
            assert len(data) == 1800, len(data)
            scan = []
            for i, d in enumerate(data):
                if d > 0:
                    a = math.radians(hack + 360*i/1800.0)
                    d /= 1000.0
                    x, y = d * math.cos(a), d * math.sin(a)
                    scan.append((x, y))
            yield scan
            hack += 1


def nearest(scan1, scan2):
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

    print(a, b, c, d)
    m = np.array([a, b, c, d]).reshape(2, 2)
    U, S, V = np.linalg.svd(m)
#    rot = U * V.T
#    rot = U * V
    rot = np.matmul(U, V)
#    rot = np.array([[1, 0], [0, 1]])
    trans = np.array([xc1, yc1]) - np.matmul(rot, np.array([xc2, yc2]).T)
    print(trans, rot)
    return trans, rot


def draw_scans(scan1, scan2, pairs=None, filename=None):
    plt.clf()
    plt.plot(*zip(*scan1), '-x')
    plt.plot(*zip(*scan2), '-o')
    if pairs is not None:
        for (x1, y1), (x2, y2) in pairs:
            plt.plot([x1, x2], [y1, y2], 'g')
#    plt.show()
    if filename is not None:
        plt.savefig(filename)

def my_icp(scan1, scan2):
    for i in range(20):
        pairs = nearest(scan1, scan2)
        draw_scans(scan1, scan2, pairs, filename=f'image{i:02}.png')
        trans, rot = transform(pairs)
        scan2b = np.matmul(np.array(scan2), rot.T) + trans.T
        scan2 = scan2b

    draw_scans(scan1, scan2)


def run_icp(filename):
    prev = None
    for i, scan in enumerate(scan_gen(filename)):
        if i % 10 != 0:
            continue
        if prev is not None:
            my_icp(prev, scan)
            break
        prev = scan


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('logfile', help='OSGAR logfile')
    args = parser.parse_args()
    run_icp(args.logfile)

    """
    if prev is not None:
        pc_fix = PointCloud(prev, columns=["x", "y", "z"])
        pc_mov = PointCloud(scan, columns=["x", "y", "z"])

        # Create simpleICP object, add point clouds, and run algorithm!
        icp = SimpleICP()
        icp.add_point_clouds(pc_fix, pc_mov)
        try:
            H, X_mov_transformed, rigid_body_transformation_params, distance_residuals = icp.run(max_overlap_distance=1)
            print(H)
        except SimpleICPException:
            print('Exception')"""
