#!/usr/bin/python
"""
  Plot data
  usage:
       ./plot.py <text data>
"""
import sys
import math
import matplotlib.pyplot as plt


def scatter(arr1, arr2):
    "mix two time based arrays based on first axis (time)"
    i, j = 0, 0
    arr = []
    while i < len(arr1) and j < len(arr2):
        if abs(arr1[i][0] - arr2[j][0]) < 0.001:
            arr.append( (arr1[i][1], arr2[j][1]) )
            i += 1
            j += 1
        elif arr1[i][0] < arr2[j][0]:
            i += 1
        else:
            j += 1
    return arr


def get_arr0(filename):
    arr = []
    enc_arr = []
    wheel_arr = []
    prev_wheel = 0
    for line in open(filename):
        if 'ENC' in line:
            prefix_cmd, t, x, y = line.split()
            enc_arr.append((float(t), (int(x), int(y))))
        if 'WHEEL' in line:
            prefix_cmd, t, angle, desired = line.split()
            if desired != 'None':
#                wheel_arr.append((float(t), (int(angle), float(desired))))
                wheel_arr.append((float(t), int(angle)))
                prev_wheel = float(desired)
            else:
#                wheel_arr.append((float(t), (int(angle), prev_wheel)))
                wheel_arr.append((float(t), int(angle)))
        if 'xSPEED' in line:
            prefix_cmd, t, raw, avr = line.split()
            arr.append((t, (raw, avr)))
        if 'xGAS' in line:
            prefix_cmd, gas, desired = line.split()
            arr.append((len(arr), (int(gas), int(desired))))
        if 'xSYNC' in line:
            gas = line.split()[-1]
            if gas != '0':
                arr.append((len(arr), float(gas)))

    for prev, curr in zip(enc_arr, enc_arr[6:]):
        L, R = curr[1][0]-prev[1][0], curr[1][1]-prev[1][1]
#        arr.append(( (prev[0]+curr[0])/2, (L, R)))
        if abs(L + R) > 20:
            arr.append(( (prev[0]+curr[0])/2, (R-L)/(L+R) ))
#    return scatter(arr, wheel_arr)
    return scatter(wheel_arr, arr)


def get_arr1(filename):
    arr = []
    for line in open(filename):
        if 'DRIVER_DIST' in line:
            prefix_cmd, t, dist = line.split()
            arr.append((t, dist))
    return arr


def get_arr(filename):
    arr = []
    for line in open(filename):
        if 'LASER_CONE' in line:
            prefix_cmd, t, raw_angle, raw_dist, raw_width, color = line.split()
            arr.append((t, int(raw_angle)/2 - 135, color))
    return arr


def draw(arr):
#    plt.plot(arr, 'o-', linewidth=2)
    x = [x for (x, _) in arr]
    y = [y for (_, y) in arr]
    plt.plot(x, y, 'o', linewidth=2)
#    plt.xlabel('raw steering')
#    plt.ylabel('encoders normalized difference')
    plt.xlabel('time (sec)')
#    plt.ylabel('signed distance (meters)')
    plt.ylabel('laser angle (deg)')

#    z = []
#    for i in xrange(len(y)):
#        z.append(sum(y[i-50:i+50])/100.0)
#    plt.plot(x, z, 'o-', linewidth=2)

    plt.show()



def draw3(arr):
    for color in ['FF0000', '00FF00', '0000FF', 'FFFFFF',
                  'FF8000', '808080']:
        x = [x for (x, _, c) in arr if c == color]
        y = [y for (_, y, c) in arr if c == color]
        if color == 'FFFFFF':
            color = '000000'
        plt.plot(x, y, 'o', linewidth=2, color='#'+color)
    plt.xlabel('time (sec)')
    plt.ylabel('laser angle (deg)')
    plt.show()


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(2)
    filename = sys.argv[1]
    arr = get_arr(filename)
    draw3(arr)


# vim: expandtab sw=4 ts=4 

