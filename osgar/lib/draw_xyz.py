"""
    Draw 3D map
"""
import math

import matplotlib.pyplot as plt


def draw3d(filename):
    x, y = [], []
    arr = []
    for line in open(filename):
        s = line.split()
        assert len(s) == 3, s
        arr.append(tuple([float(v) for v in s]))

    # TODO equal axis
    # TODO grid 0.1m
    floor = [(x, y) for x, y, z in arr if abs(z) < 0.1]
    x = [x for (x, _) in floor]
    y = [y for (_, y) in floor]
    plt.plot(x, y, 'bo')

    obs = [(x, y) for x, y, z in arr if z > 0.1]
    x = [x for (x, _) in obs]
    y = [y for (_, y) in obs]
    plt.plot(x, y, 'rx')

    hole = [(x, y) for x, y, z in arr if z < -0.1]
    x = [x for (x, _) in hole]
    y = [y for (_, y) in hole]
    plt.plot(x, y, 'ko')


if __name__ == '__main__':
    import argparse

    arg_parser = argparse.ArgumentParser(description='Draw 3D map')
    arg_parser.add_argument('filename', help='input text file')
    args = arg_parser.parse_args()

    draw3d(args.filename)
    plt.axes().set_aspect('equal', 'datalim')
    plt.show()

# vim: expandtab sw=4 ts=4
