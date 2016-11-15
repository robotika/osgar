#!/usr/bin/python
"""
  Convert data referenced in metalog to viewer format
  usage:
       ./meta2view.py <metalog>
"""
import math
import os
import sys


def laser_gen(filename):
    print filename
    for line in open(filename):
        if line.startswith('['):
            arr = eval(line)
            yield arr

        # TODO handle timestamps

def dump_pose(out, pose):
    out.write('RefPose 1 {} {} {}\n'.format(*pose))
    out.write('Poses\n')
    out.write('{} {} {}\n'.format(*pose))


def dump_laser(out, data):
    assert len(data) == 541, len(data)
    out.write('Geometry')
    for angle in xrange(-135, 135, 5):
        s = 0, 0, math.radians(angle)
        out.write(' %.2f %.2f %.4f' % s)
    out.write('\n')

    pose = (0, 0, 0)
    out.write('Ranger %.2f %.2f %.4f ' % pose)
    for i in xrange(0, 540, 10):
        dist = data[i]/1000.0
        out.write(' %.3f' % dist)
    out.write('\n')


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print __doc__
        sys.exit(2)

    meta_filename = sys.argv[1]
    meta_dir = os.path.split(meta_filename)[0]
    laser = None
    for line in open(meta_filename):
        print line
        if line.startswith('laser:'):
            filename = os.path.split(line.split()[1])[1]
            laser = laser_gen(os.path.join(meta_dir, filename))

    out = open('view.log', 'w')
    pose = (0, 0, 0)
    if laser is not None:
        for data in laser:
            dump_pose(out, pose)
            dump_laser(out, data)

# vim: expandtab sw=4 ts=4 

