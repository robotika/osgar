#!/usr/bin/python
"""
  Convert data referenced in metalog to viewer format
  usage:
       ./meta2view.py <metalog>
"""
import math
import os
import struct
import sys

sys.path.append(os.path.join(os.path.split(__file__)[0], '..'))
from can import parseFileG


def can_gen(filename):
    m = {
            0x284: ('encoders', 'hh'),
            0x182: ('steering', 'h')
        }
    can = parseFileG(filename)
    ret = {}
    for msg in can:
        io_dir, msg_type, data = msg
        if io_dir == 1:
            if msg_type == 0x80:
                yield ret
                ret = {}
            elif msg_type in m:
                name, str_format = m[msg_type]
                val = struct.unpack(str_format, ''.join([chr(c) for c in data]))
                if len(val) == 1:
                    val = val[0]
                ret[name] = val
            # print('%X - %s' % (msg_type, data))


def laser_gen(filename):
    print(filename)
    num = None
    for line in open(filename):
        if line.startswith('['):
            arr = eval(line)
            yield num, arr
        else:
            num = int(line)

        # TODO handle timestamps

def dump_pose(out, pose):
    out.write('RefPose 1 {} {} {}\n'.format(*pose))
    out.write('Poses\n')
    out.write('{} {} {}\n'.format(*pose))


def dump_laser(out, data):
    assert len(data) == 541, len(data)
    step = 2
    out.write('Geometry')
    for angle in xrange(-135, 135, step/2):
        s = 0, 0, math.radians(angle)
        out.write(' %.2f %.2f %.4f' % s)
    out.write('\n')

    pose = (0, 0, 0)
    out.write('Ranger %.2f %.2f %.4f ' % pose)
    for i in xrange(0, 540, step):
        dist = data[i]/1000.0
        out.write(' %.3f' % dist)
    out.write('\n')


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(2)

    meta_filename = sys.argv[1]
    meta_dir = os.path.split(meta_filename)[0]
    can = None
    laser = None
    for line in open(meta_filename):
        print(line)
        if line.startswith('can:'):
            filename = os.path.split(line.split()[1])[1]
            can = can_gen(os.path.join(meta_dir, filename))

        if line.startswith('laser:'):
            filename = os.path.split(line.split()[1])[1]
            laser = laser_gen(os.path.join(meta_dir, filename))

    out = open('view.log', 'w')
    pose = [0, 0, 0]
    enc = (0, 0)
    # unknown start
    for i in xrange(80):
        can.next()
    if laser is not None:
        for num, data in laser:
            for i in xrange(num):
                sensors = can.next()
                if 'encoders' in sensors:
                    encL, encR = sensors['encoders']
                    dist = ((encL - enc[0]) + (encR - enc[1])) * 0.01
                    angle = sensors['steering'] * 0.001

                    print dist, sensors
                    enc = (encL, encR)
                    x, y, heading = pose
                    x += math.cos(heading) * dist
                    y += math.sin(heading) * dist
                    heading += angle
                    pose = x, y, heading
                dump_pose(out, pose)
            dump_laser(out, data)

# vim: expandtab sw=4 ts=4 

