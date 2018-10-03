#!/usr/bin/python
"""
  Convert logfile to AVI video
"""
try:
    import cv2
except ImportError:
    print('\nERROR: Please install OpenCV\n    pip install opencv-python\n')

try:
    import numpy as np
except ImportError:
    print('\nERROR: Please install numpy\n    pip install numpy\n')

from osgar.logger import LogReader, lookup_stream_id
from osgar.lib.serialize import deserialize


def create_video(logfile, outfile):
    assert outfile.endswith(".avi"), outFilename
    only_stream = lookup_stream_id(logfile, 'camera.raw')
    with LogReader(logfile) as log:
        writer = None
        for timestamp, stream_id, data in log.read_gen(only_stream):
            buf = deserialize(data)
            img = cv2.imdecode(np.fromstring(buf, dtype=np.uint8), 1)
            if writer is None:
                height, width = img.shape[:2]
                writer = cv2.VideoWriter(outfile,
                                         cv2.VideoWriter_fourcc('F', 'M', 'P', '4'), 
                                         5, 
                                         (width, height))
            writer.write(img)
        writer.release()


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description='Convert logfile to AVI video')
    parser.add_argument('logfile', help='recorded log file')
    parser.add_argument('--out', '-o', help='output AVI file', default='out.avi')
    args = parser.parse_args()

    create_video(args.logfile, args.out)

# vim: expandtab sw=4 ts=4 

