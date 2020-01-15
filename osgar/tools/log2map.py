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
    parser.add_argument('--artf', help='stream ID with artifacts XYZ')
    args = parser.parse_args()

    out_filename = args.out
    if out_filename is None:
        out_filename = os.path.splitext(args.logfile)[0] + '.jpg'

    pts = []
    artf = []
    pose_id = lookup_stream_id(args.logfile, args.pose2d)
    streams = [pose_id]
    artf_id = None
    if args.artf is not None:
        artf_id = lookup_stream_id(args.logfile, args.artf)
        streams.append(artf_id)
    for dt, channel, data in LogReader(args.logfile, only_stream_id=streams):
        if channel == pose_id:
            pose = deserialize(data)
            pts.append(pose[:2])
        elif channel == artf_id:
            arr = deserialize(data)
            artf.extend(arr)
    arr = np.array(pts)
    min_x, min_y = np.min(arr, axis=0)
    max_x, max_y = np.max(arr, axis=0)

    if len(artf) > 0:
        arr_x = [x for __, x, __, __ in artf]
        arr_y = [y for __, __, y, __ in artf]
        min_x = min(min_x, min(arr_x))
        max_x = max(max_x, max(arr_x))
        min_y = min(min_y, min(arr_y))
        max_y = max(max_y, max(arr_y))

    width_px = 2*BORDER_PX + int(SCALE*(max_x - min_x)/1000.0)
    height_px = 2*BORDER_PX + int(SCALE*(max_y - min_y)/1000.0)
    print(width_px, height_px)
    world = np.zeros((height_px, width_px), dtype=np.uint8)
    for x, y in arr:
        px = int(SCALE*(x - min_x)/1000.0) + BORDER_PX
        py = int(SCALE*(y - min_y)/1000.0) + BORDER_PX
        world[height_px - py - 1, px] = 255
    for name, x, y, z in artf:
        px = int(SCALE*(x - min_x)/1000.0) + BORDER_PX
        py = int(SCALE*(y - min_y)/1000.0) + BORDER_PX
        world[height_px - py - 1, px] = 1
        world[height_px - py - 2, px - 1] = 1
        world[height_px - py - 2, px + 1] = 1
        world[height_px - py, px - 1] = 1
        world[height_px - py, px + 1] = 1

    user_color_map = np.zeros((256, 1, 3), dtype=np.uint8)
    user_color_map[0] = (0, 0, 0)
    user_color_map[1] = (255, 255, 255)
    user_color_map[255] = (50, 50, 255)  # BGR -> Red
    cimg = cv2.applyColorMap(world, user_color_map)
    cv2.imwrite(out_filename, cimg)


if __name__ == "__main__":
    main()

# vim: expandtab sw=4 ts=4 
