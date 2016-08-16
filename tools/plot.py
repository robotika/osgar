#!/usr/bin/python
"""
  Plot data
  usage:
       ./plot.py <text data>
"""
import sys
import math
import matplotlib.pyplot as plt


def get_arr(filename):
    arr = []
    for line in open(filename):
        if 'ENC' in line:
            prefix_cmd, x, y = line.split()
            arr.append((int(x), int(y)))
    return arr

def draw(arr):
    plt.plot(arr, 'o-', linewidth=2)
    plt.show()


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print __doc__
        sys.exit(2)
    filename = sys.argv[1]
    arr = get_arr(filename)
    draw(arr)


# vim: expandtab sw=4 ts=4 

