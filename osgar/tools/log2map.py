"""
  Create map (gridmap image)
"""
import os.path

import cv2  # for video output
import numpy as np  # faster depth data processing

from osgar.logger import LogReader, lookup_stream_names, lookup_stream_id
from osgar.lib.serialize import deserialize

SCALE = 10  # 1 pixel is 1dm
BORDER_PX = 10  # extra border


def main():
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('logfile', help='recorded log file')
    parser.add_argument('--out', help='map output file')
    parser.add_argument('--pose2d', help='stream ID for pose2d messages', default='app.pose2d')
    args = parser.parse_args()

    out_filename = args.out
    if out_filename is None:
        out_filename = os.path.join(os.path.dirname(args.logfile), 'world.jpg')

    pts = []
    for dt, channel, data in LogReader(args.logfile, only_stream_id=lookup_stream_id(args.logfile, args.pose2d)):
        pose = deserialize(data)
        pts.append(pose[:2])
    arr = np.array(pts)
    min_x, min_y = np.min(arr, axis=0)
    max_x, max_y = np.max(arr, axis=0)

    width_px = 2*BORDER_PX + int(SCALE*(max_x - min_x)/1000.0)
    height_px = 2*BORDER_PX + int(SCALE*(max_y - min_y)/1000.0)
    print(width_px, height_px)
    world = np.zeros((height_px, width_px))
    for x, y in arr:
        px = int(SCALE*(x - min_x)/1000.0) + BORDER_PX
        py = int(SCALE*(y - min_y)/1000.0) + BORDER_PX
        world[height_px - py - 1, px] = 255
    cv2.imwrite(out_filename, world)


if __name__ == "__main__":
    main()

# vim: expandtab sw=4 ts=4 
