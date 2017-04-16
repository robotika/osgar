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
from logparser2 import sensor_gen

FRONT_REAR_DIST = 1.3
LEFT_WHEEL_DIST_OFFSET = 0.4  # from central axis
TURN_ANGLE_OFFSET = math.radians(5.5)
TURN_SCALE = 0.0041

# (252, 257) corresponds to 339cm
ENC_SCALE = 2*3.39/float(252 + 257)

LASER_OFFSET = (1.78, 0.0, 0.39)

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
    timestamp = None
    for line in open(filename):
        if line.startswith('['):
            arr = eval(line)
            yield num, timestamp, arr
        else:
            num = int(line.split()[0])  # support for offset + timestamp format
            if len(line.split()) > 1:
                timestamp = float(line.split()[1])

        # TODO handle timestamps

def dump_pose(out, pose):
    out.write('RefPose 1 {} {} {}\n'.format(*pose))
    out.write('Poses\n')
    out.write('{} {} {}\n'.format(*pose))


def dump_laser(out, pose, data):
    assert len(data) == 541, len(data)
    step = 2
    out.write('Geometry')
    for angle in xrange(-135, 135, step/2):
        s = 0, 0, math.radians(angle)
        out.write(' %.2f %.2f %.4f' % s)
    out.write('\n')

    x, y, heading = pose
    laser_pose = x + math.cos(heading)*LASER_OFFSET[0], y + math.sin(heading)*LASER_OFFSET[0], heading
    out.write('Ranger %.2f %.2f %.4f ' % laser_pose)
    for i in xrange(0, 540, step):
        dist = data[i]/1000.0
        out.write(' %.3f' % dist)
    out.write('\n')


def dump_camera(out, img_dir, data):
    filename = os.path.join(img_dir, data[0][5:])
    out.write( "Image %s\n" % filename )
    out.write( "ImageResult %s\n" % data[1]) 


def main():
    meta_filename = sys.argv[1]
    meta_dir = os.path.split(meta_filename)[0]
    camera_gen = sensor_gen(meta_filename, ['camera'])
    can = None
    laser = None
    camera_timestamp, __, camera_data = camera_gen.next()
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
    enc = [0, 0]
    # unknown start
    for i in xrange(80):
        can.next()
    if laser is not None:
        for num, laser_time, data in laser:
            if camera_timestamp is not None and laser_time > camera_timestamp:
                dump_camera(out, meta_dir, camera_data)
                try:
                    camera_timestamp, __, camera_data = camera_gen.next()
                except StopIteration:
                    camera_timestamp = None
            for i in xrange(num):
                sensors = can.next()
                if 'encoders' in sensors:
                    encL, encR = sensors['encoders']
                    if abs(encL - enc[0]) > 255:
                        enc[0] = encL
                    if abs(encR - enc[1]) > 255:
                        enc[1] = encR
                    distL = (encL - enc[0]) * ENC_SCALE
                    angle = sensors['steering'] * TURN_SCALE + TURN_ANGLE_OFFSET  # radians
                    
                    dh = math.sin(angle) * distL / FRONT_REAR_DIST
                    dist = math.cos(angle) * distL + LEFT_WHEEL_DIST_OFFSET * dh 
                    
                    enc = [encL, encR]
                    x, y, heading = pose
                    x += math.cos(heading) * dist
                    y += math.sin(heading) * dist
                    heading += dh
                    pose = x, y, heading
                dump_pose(out, pose)
            dump_laser(out, pose, data)
    return pose

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(2)
    pose = main()
    print TURN_SCALE, pose
    print math.degrees(pose[2])/360.0, int(math.degrees(pose[2])) % 360

# vim: expandtab sw=4 ts=4 

