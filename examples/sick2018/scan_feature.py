"""
   Laser Feature Extractor
"""
import math
import numpy as np

ANGULAR_RESOLUTION = math.radians(1/3)

DEG_STEP = 5


def scan2xy(scan):
    a = ( len(scan)//2 - np.arange(len(scan)) ) * ANGULAR_RESOLUTION
    x = np.cos(a) * scan
    y = np.sin(a) * scan
    return x, y


def filter_scan(scan):
    scan = np.array(scan)
    scan_diff = np.absolute(np.diff(scan))
    binary = scan_diff > 150 #diff limint
    edges_arg = np.argwhere(binary)
    edges_num = len(edges_arg)
    for ii in range(edges_num + 1):
        if ii == 0:
            id_0 = 0
        else:
            id_0 = edges_arg[ii-1, 0] + 1
            
        if ii != edges_num:
            id_1 = edges_arg[ii, 0] + 1
        else:
            id_1 = edges_num
        
        if id_1 - id_0 < 3: # series smaller to 2 are removed
            scan[id_0:id_1] = 4000 # or other favourite number
    return scan


def scan_split(scan, max_diff, min_len=10):
    arr = []
    prev = scan[0]
    start_i = 0
    for i, dist in enumerate(scan[1:], start=1):
        diff = abs(dist - prev)
        if diff > max_diff:
            if i - start_i >= min_len:
                arr.append((start_i, i-1))
            start_i = i
        prev = dist
    arr.append((start_i, i))
    return arr


def get_scan_diff(scan):
    prev = None
    arr = []
    for i in range(len(scan)):
        dist = scan[i]
        if prev is not None:
            diff = abs(dist - prev)
            arr.append(diff)
        prev = dist
    return arr


def extract_features(scan):
    pairs = scan_split(scan[135:-135], max_diff=20)
    pairs = [(f+135, t+135) for f, t in pairs]

    seek = len(scan)//2
    for f, t in pairs:
        if f <= seek <= t:
            return [(f, t)]
    return []


def coord_xy(i, scan):
    if i < 0 or i >= len(scan):
        return 0, 0
    angle = (len(scan)//2 - i) * ANGULAR_RESOLUTION
    # TODO check overflow
    dist = scan[i]
    return math.cos(angle) * dist, math.sin(angle) * dist


def is_box_center(i, scan, verbose=False):
    """
    We expect to to see box in angles -45deg to 45deg. Distance at given
    point gives us angles. Scan is expected raw, i.e. in mm (removed 0).
    """
    near_line = 10

    dist = scan[i]
    if dist <= 300:  # arc sin domain for limit
        return False  # should be None??
    # max angle is for right-angled triangle 20cm
    angle = math.asin(200/dist)
#    print(math.degrees(angle))
    angle_step = int((angle/ANGULAR_RESOLUTION)/2)
    if i < 4 * angle_step or i + 4 * angle_step >= len(scan):
        return False
    if scan[i - angle_step] == 0 or scan[i + angle_step] == 0:
        return False
    if angle_step < 1:
        return False
    if verbose:
        print('angle_step =', angle_step)
    x1, y1 = coord_xy(i - angle_step, scan)
    x2, y2 = coord_xy(i + angle_step, scan)

    # ax + by + c = 0
    a = -(y2 - y1)
    b = -(x1 - x2)
    d = math.hypot(a, b)
    a /= d
    b /= d
    c = - a * x1 - b * y1
    if verbose:
        print('abc =', a, b, c)

    # TODO verify intermediate points
    x, y = coord_xy(i, scan)
    d = abs(a*x + b*y + c)
    if verbose:
        print(d)
    if d > near_line:
        return False

    x, y = coord_xy(i - 3 * angle_step, scan)
    d1 = a*x + b*y + c

    x, y = coord_xy(i + 3 * angle_step, scan)
    d2 = a*x + b*y + c
    if verbose:
        print(d1, d2)

    x, y = coord_xy(i - 4 * angle_step, scan)
    d3 = a*x + b*y + c

    x, y = coord_xy(i + 4 * angle_step, scan)
    d4 = a*x + b*y + c
    if verbose:
        print(d3, d4)

    count = int(200 < d1 < 400) + int(200 < d2 < 400)
    count += int(200 < d3 < 400) + int(200 < d4 < 400)
    return count >= 2


def detect_box(scan):
    box = []
    for i, __ in enumerate(scan[135:-135], start=135):
        if is_box_center(i, scan):
            box.append(i)
    if len(box) < 1:
        return None
    return box[len(box)//2]


def find_transporter(small_scan):
    DIST_GAP = 500
#    print(list(small_scan))

    tmp = small_scan
    i = np.argmin(tmp)
    center_i = i
    while i > 0 and tmp[i] <= tmp[center_i] + DIST_GAP:
        i -= 1
    left_i = i + 1
    
    i = center_i
    while i < len(tmp) and tmp[i] <= tmp[center_i] + DIST_GAP:
        i += 1
    right_i = i - 1

    dist = tmp[center_i]/1000.0
    size = dist * math.radians((right_i - left_i + 1)*DEG_STEP)
#    print(size)
#    print(list(small_scan[left_i:right_i+1]))
    if size > 1.0:
        return None
    return left_i, right_i


def shift_polar(angle, dist, offset_y):
    x, y = math.cos(angle) * dist, math.sin(angle) * dist
    ret_angle = math.atan2(y - offset_y, x)
    ret_dist = math.hypot(x, y - offset_y)
    return ret_angle, ret_dist


def detect_transporter(scan, offset_y):
    # expected raw scan in mm, with 0 as timeout
    assert len(scan) == 811, len(scan)
    scan = np.array(scan[135:-136])  # 180 deg only
    assert len(scan) == 540, len(scan)
    mask = scan < 10
    scan[mask] = 20000
    small = scan.reshape((180//DEG_STEP, DEG_STEP*3))
    tmp = np.min(small, axis=1)

    ret = find_transporter(tmp)
    if ret is None:
        return None
    left_i, right_i = ret

    angle = math.radians(90 - DEG_STEP*(left_i + right_i)/2)
    dist = tmp[(left_i + right_i)//2]/1000.0
    return shift_polar(angle, dist, offset_y=offset_y)
    

def draw_xy(scan, pairs, scan2 = None):
    step_angle = ANGULAR_RESOLUTION
    arr_x, arr_y = [], []
    box_x, box_y = [], []
    for i, dist in enumerate(scan):
        angle = (len(scan)//2 - i) * step_angle
        x, y = math.cos(angle) * dist, math.sin(angle) * dist
        arr_x.append(x)
        arr_y.append(y)
        if is_box_center(i, scan):
            box_x.append(x)
            box_y.append(y)
    plt.plot(arr_x, arr_y, '.', linewidth=2)
    plt.plot(box_x, box_y, 'ro', linewidth=2)

    for f, t in pairs:
        plt.plot([arr_x[f], arr_x[t]], [arr_y[f], arr_y[t]], 'o-', linewidth=2)

    if scan2 is not None:
        x, y, = scan2xy(scan2)
        plt.plot(x, y, '+k')

    plt.axis('equal')
    plt.show()


if __name__ == "__main__":
    import argparse
    import matplotlib.pyplot as plt
    from osgar.logger import LogReader, lookup_stream_id
    from osgar.lib.serialize import deserialize

    parser = argparse.ArgumentParser(description='Extract features in laser scan')
    parser.add_argument('filename', help='input log file')
    parser.add_argument('--verbose', '-v', help="verbose mode", action='store_true')
    parser.add_argument('--draw', '-d', help="draw result", action='store_true')
    parser.add_argument('--index', '-i', help="scan index", type=int)
    parser.add_argument('--transporter', help="detect_transporter", action='store_true')
    args = parser.parse_args()

    filename = args.filename
    only_stream = lookup_stream_id(filename, 'lidar.scan')
    index = args.index
    offset_y = 0

    with LogReader(filename, only_stream_id=only_stream) as log:
        for ind, row in enumerate(log):
            if index is not None and ind < index:
                continue
            timestamp, stream_id, data = row
            scan = deserialize(data)

            if args.transporter:
                trans = detect_transporter(scan, offset_y)
                if trans is None:
                    print('%d\tNone' % ind)
                else:
                    print('%d\t%.1f\t%.3f' % 
                          (ind, math.degrees(trans[0]), trans[1]))
            else:
                pairs = scan_split(scan[135:-135], max_diff=20)
                if args.verbose:
                    for f, t in pairs:
                        print(f, t)
                        print(scan[135+f:135+t])
                pairs = [(f+135, t+135) for f, t in pairs]
#            pairs = extract_features(scan)
#            is_box_center(441, scan, verbose=True)
#            print(ind, detect_box(scan))
            if args.draw:
#                draw_xy(scan, pairs)
                scan2 = filter_scan(scan)
                draw_xy(scan, pairs, scan2)
                break
            if index is not None:
                break


# vim: expandtab sw=4 ts=4

