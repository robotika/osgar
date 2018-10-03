#!/usr/bin/python
"""
  Convert directory of images into AVI video
    usage:
         ./log2video.py <logfile> <output AVI file>
"""

import sys
import os

import cv2
import numpy as np

from osgar.logger import LogReader, lookup_stream_id
from osgar.lib.serialize import deserialize


def create_video(logfile, outFilename):
    assert outFilename.endswith(".avi"), outFilename
    only_stream = lookup_stream_id(logfile, 'camera.raw')
    writer = cv2.VideoWriter( outFilename, cv2.VideoWriter_fourcc('F', 'M', 'P', '4'), 5, (640,512) ) 
#    writer = cv2.VideoWriter( outFilename, cv2.VideoWriter_fourcc('F', 'M', 'P', '4'), 5, (1024,768) ) 
    with LogReader(logfile) as log:
        for timestamp, stream_id, data in log.read_gen(only_stream):
            buf = deserialize(data)
            img = cv2.imdecode(np.fromstring(buf, dtype=np.uint8), 1)
            writer.write(img)
    writer.release()


if __name__ == "__main__": 
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(-1)
    path = sys.argv[1]
    output_file = sys.argv[2]
    create_video(path, output_file)

# vim: expandtab sw=4 ts=4 

