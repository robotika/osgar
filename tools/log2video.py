#!/usr/bin/python
"""
  Convert directory of images into AVI video
    usage:
         ./log2video.py <log directory> <output AVI file>
"""

import sys
import os
import cv2


def video(path, outFilename):
    "create demo video"
    assert outFilename.endswith(".avi"), outFilename
#    writer = cv2.VideoWriter( outFilename, cv2.cv.CV_FOURCC('F', 'M', 'P', '4'), 4, (640,512) ) 
    writer = cv2.VideoWriter( outFilename, cv2.cv.CV_FOURCC('F', 'M', 'P', '4'), 5, (1024,768) ) 
    for (dirpath, dirnames, filenames) in os.walk(path):
        for name in filenames:
            if name.endswith(".jpg"):
                print(name)
                img = cv2.imread(dirpath + os.sep + name )
                writer.write( img ) 
    writer.release()


if __name__ == "__main__": 
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(-1)
    path = sys.argv[1]
    output_file = sys.argv[2]
    video(path, output_file)

#-------------------------------------------------------------------
# vim: expandtab sw=4 ts=4 

