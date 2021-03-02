"""
Simple Path Planner
"""


def find_path(img, start, finish, verbose=False):
    max_x, max_y = img.shape

    if not (0 <= start[0] < max_x and 0 <=start[1] < max_y):
        return None

    if not img[start[1]][start[0]]:
        return None

    queue = [(start, None)]

    expanded = set()
    prev = {}


    ret = None
    while len(queue) > 0:
        head, queue = queue[0], queue[1:]
        if verbose:
            print(head)
        if head[0] in expanded:
            continue
        expanded.add(head[0])

        if head[0] in finish:
            ret = [head[0]]
            p = head[1]
            while p is not None:
                ret.append(p)
                p = prev[p]
            ret = list(reversed(ret))
            break

        assert head[0] not in prev, head
        prev[head[0]] = head[1]

        x, y = head[0]
        if x + 1 < max_x and img[y][x + 1] and (x + 1, y) not in expanded:
            queue.append(((x + 1, y), (x, y)))
        if x > 0 and img[y][x - 1] and (x - 1, y) not in expanded:
            queue.append(((x - 1, y), (x, y)))
        if y + 1 < max_y and img[y + 1][x] and (x, y + 1) not in expanded:
            queue.append(((x, y + 1), (x, y)))
        if y > 0 and img[y - 1][x] and (x, y - 1) not in expanded:
            queue.append(((x, y - 1), (x, y)))

    return ret

# vim: expandtab sw=4 ts=4
