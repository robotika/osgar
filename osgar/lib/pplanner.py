"""
Simple Path Planner
"""


def find_path(img, start, finish, verbose=False):
    """
    Find path in 2D image using Dijstra algorithm and 4-neighbor connecitivity
    https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm
    :param img: 2D binary image
    :param start: pair (x, y)
    :param finish: list of pairs (x, y)
    :param verbose: true for debugging
    :return: path (list of coordinates) or None if path not found
    """
    max_x, max_y = img.shape

    if not (0 <= start[0] < max_x and 0 <=start[1] < max_y):
        return None

    if not img[start[1]][start[0]]:
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

        x, y = node
        if x + 1 < max_x and img[y][x + 1] and (x + 1, y) not in expanded:
            queue.append(((x + 1, y), (x, y)))
        if x > 0 and img[y][x - 1] and (x - 1, y) not in expanded:
            queue.append(((x - 1, y), (x, y)))
        if y + 1 < max_y and img[y + 1][x] and (x, y + 1) not in expanded:
            queue.append(((x, y + 1), (x, y)))
        if y > 0 and img[y - 1][x] and (x, y - 1) not in expanded:
            queue.append(((x, y - 1), (x, y)))

    return None

# vim: expandtab sw=4 ts=4
