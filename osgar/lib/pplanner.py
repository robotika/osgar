"""
Simple Path Planner
"""

import numpy as np


def find_path_old(img, start, finish, verbose=False):
    """
    Find path in 2D image using Dijstra algorithm and 4-neighbor connecitivity
    https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm
    :param img: 2D binary image
    :param start: pair (x, y)
    :param finish: list of pairs (x, y)
    :param verbose: true for debugging
    :return: path (list of coordinates) or None if path not found
    """
    if len(img.shape) == 2:
        # old 2D images - add extra dimension
        img = np.expand_dims(img, axis=2)
        start = (start[0], start[1], 0)
        finish = [(x, y, 0) for x, y in finish]
    max_x, max_y, max_z = img.shape

    if not (0 <= start[0] < max_x and 0 <= start[1] < max_y and 0 <= start[2] < max_z):
        return None

    if not img[start[1]][start[0]][start[2]]:
        return None

    queue = [(start, None)]  # start location has no predecessor

    expanded = set()
    prev = {}


    while len(queue) > 0:
        (node, node_prev), queue = queue[0], queue[1:]
        if verbose:
            print((node, node_prev))
        if node in expanded:
            continue
        expanded.add(node)

        if node in finish:
            ret = [node]
            p = node_prev
            while p is not None:
                ret.append(p)
                p = prev[p]
            ret = list(reversed(ret))
            return ret

        assert node not in prev, (node, node_prev)
        prev[node] = node_prev

        x, y, z = node
        if x + 1 < max_x and img[y][x + 1][z] and (x + 1, y, z) not in expanded:
            queue.append(((x + 1, y, z), (x, y, z)))
        if x > 0 and img[y][x - 1][z] and (x - 1, y, z) not in expanded:
            queue.append(((x - 1, y, z), (x, y, z)))
        if y + 1 < max_y and img[y + 1][x][z] and (x, y + 1, z) not in expanded:
            queue.append(((x, y + 1, z), (x, y, z)))
        if y > 0 and img[y - 1][x][z] and (x, y - 1, z) not in expanded:
            queue.append(((x, y - 1, z), (x, y, z)))
        if z + 1 < max_z and img[y][x][z + 1] and (x, y, z + 1) not in expanded:
            queue.append(((x, y, z + 1), (x, y, z)))
        if z > 0 and img[y][x][z - 1] and (x, y, z - 1) not in expanded:
            queue.append(((x, y, z - 1), (x, y, z)))

    return None


def find_path(img, start, finish, yaw_deg=None, verbose=False):
    """
    Follow wall in multi-resolution image
    :param img: binary image
    :param start: pair (x, y, z)
    :param finish: list of pairs (x, y, z)
    :param yaw_deg: yaw in degrees (0, 90, 180, 270)
    :param verbose: true for debugging
    :return: path (list of coordinates) or None if path not found
    """
    max_x, max_y, max_z = img.shape

    if not (0 <= start[0] < max_x and 0 <= start[1] < max_y and 0 <= start[2] < max_z):
        return None

    if yaw_deg is not None:
        assert yaw_deg in [0, 90, 180, 270], yaw_deg

    # find nearest wall in 4 directions
    x, y, z = start
    z = 4  # hack - force height 2m (absolute level)
    if not img[y][x][z]:
        return None  # in the wall, no path found
    for i in range(10):
        if (yaw_deg == 270 or yaw_deg is None) and (x + i >= max_x or not img[y][x + i][z]):
            node = (x + i - 1, y, z)
            yaw_deg = 270
            break
        if (yaw_deg == 90 or yaw_deg is None) and (x - i < 0 or not img[y][x - i][z]):
            node = (x - i + 1, y, z)
            yaw_deg = 90
            break
        if (yaw_deg == 180 or yaw_deg is None) and (y + i >= max_y or not img[y + i][x][z]):
            node = (x, y + i - 1, z)
            yaw_deg = 180
            break
        if (yaw_deg == 0 or yaw_deg is None) and (y - i < 0 or not img[y - i][x][z]):
            node = (x, y - i + 1, z)
            yaw_deg = 0
            break
    else:
        return None  # no nearby wall

#    if verbose:
    print(node, yaw_deg, i)

    path = []
    for i in range(10):
        path.append(node)
        if node in finish:
            break
        x, y, z = node
        assert img[y][x][z]
        if yaw_deg == 0 and y > 0 and img[y - 1][x][z]:
            node = (x, y - 1, z)
            yaw_deg = 90
        elif yaw_deg in [0, 270] and x + 1 < max_x and img[y][x + 1][z]:
            node = (x + 1, y, z)
            yaw_deg = 0
        elif yaw_deg in [0, 270, 180] and y + 1 < max_y and img[y + 1][x][z]:
            node = (x, y + 1, z)
            yaw_deg = 270
        elif x > 0 and img[y][x - 1][z]:
            node = (x - 1, y, z)
            yaw_deg = 180
        elif y > 0 and img[y - 1][x][z]:
            node = (x, y - 1, z)
            yaw_deg = 90
        elif x + 1 < max_x and img[y][x + 1][z]:
            node = (x + 1, y, z)
            yaw_deg = 0
        elif y + 1 < max_y and img[y + 1][x][z]:
            node = (x, y + 1, z)
            yaw_deg = 270
    print('MD', path)
    return path


# vim: expandtab sw=4 ts=4
