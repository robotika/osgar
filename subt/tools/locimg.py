"""
  Extract log images observing given location
"""

import os

from osgar.logger import LogReader, lookup_stream_names
from osgar.lib.serialize import deserialize
from subt.trace import distance3D


POSE3D_STREAM = 'fromrospy.pose3d'


def locimg(logfile, loc, out_dir, stream_name, radius):
    print(loc)
    names = lookup_stream_names(logfile)
    assert POSE3D_STREAM in names, (POSE3D_STREAM, names)
    assert stream_name in names, (stream_name, names)
    pose_id = names.index(POSE3D_STREAM) + 1
    camera_id = names.index(stream_name) + 1

    stream_ids = [pose_id, camera_id]
    img_count = 0
    selected = 0
    last_pose3d = None
    for dt, channel, data in LogReader(logfile, only_stream_id=stream_ids):
        data = deserialize(data)
        if channel == pose_id:
            last_pose3d = data
        elif channel == camera_id:
            if last_pose3d is not None and distance3D(last_pose3d[0], loc) < radius:
                selected += 1
                if out_dir is not None:
                    with open(os.path.join(out_dir, 'image_%04d.jpg' % img_count), 'wb') as f:
                        f.write(data)
            img_count += 1
    print(f'Images {selected}/{img_count}')


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('logfile', help='recorded log file')
    parser.add_argument('--out-dir', help='where tu dump relevant images')
    parser.add_argument('--stream', help='image source', default='logimage.image')
    parser.add_argument('--loc', help='xyz position', nargs=3, type=float, required=True)
    parser.add_argument('-r', '--radius', help='sphere radius of interest', type=float, default=5.0)
    args = parser.parse_args()

    locimg(args.logfile, args.loc, args.out_dir, args.stream, radius=args.radius)

# vim: expandtab sw=4 ts=4
