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
        if 'xENC' in line:
            prefix_cmd, t, x, y = line.split()
            arr.append((t, (int(x), int(y))))
        if 'WHEEL' in line:
            prefix_cmd, t, angle, desired = line.split()
            if desired != 'None':
                arr.append((t, (int(angle), float(desired))))
            else:
                arr.append((t, (int(angle), 0)))
        if 'xSPEED' in line:
            prefix_cmd, t, raw, avr = line.split()
            arr.append((t, (raw, avr)))
        if 'xGAS' in line:
            prefix_cmd, gas = line.split()
            arr.append((len(arr), int(gas)))
        if 'xSYNC' in line:
            gas = line.split()[-1]
            if gas != '0':
                arr.append((len(arr), float(gas)))
    return arr

def draw(arr):
#    plt.plot(arr, 'o-', linewidth=2)
    x = [x for (x, _) in arr]
    y = [y for (_, y) in arr]
    plt.plot(x, y, 'o-', linewidth=2)

#    z = []
#    for i in xrange(len(y)):
#        z.append(sum(y[i-50:i+50])/100.0)
#    plt.plot(x, z, 'o-', linewidth=2)

    plt.show()


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(2)
    filename = sys.argv[1]
    arr = get_arr(filename)
    draw(arr)


# vim: expandtab sw=4 ts=4 

