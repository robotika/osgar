"""
  Moon Skyline analysis
"""
import math

import numpy as np
import cv2  # for debug


def skyline(img):
    b = img[:,:,0]
    g = img[:,:,1]
    r = img[:,:,2]
    mask = r > 10
    return mask.argmax(axis=0)


def find_peaks(arr):
    x = arr.argmin()
    y = arr[x]
    ret = []
    if x > 0:
        ret.append((x, y))

        # try to remove the single peak
        # TODO numpy optimize
        i = x
        while i > 0 and arr[i - 1] >= arr[i]:
            i -= 1
        min_i = i

        i = x
        while i + 1 < len(arr) and arr[i] <= arr[i + 1]:
            i += 1
        max_i = i

        arr2 = arr.copy()
        arr2[min_i:max_i+1] = arr.max()

        x = arr2.argmin()
        y = arr2[x]
        if x > 0:
            ret.append((x, y))

    return ret


def draw_skyline(img, skyline):
    # TODO optimize - there is surely many times faster version, but this is just for debug
    img2 = img.copy()
    for x, y in enumerate(skyline):
        img2[y, x, 0] = 0
        img2[y, x, 1] = 255
        img2[y, x, 2] = 0

    for x, y in find_peaks(skyline):
        cv2.drawMarker(img2, (x, y), color=(255, 0, 0), markerType=cv2.MARKER_TRIANGLE_UP, thickness=3)
    return img2


def main():
    import argparse

    from osgar.logger import LogReader, lookup_stream_id
    from osgar.lib.serialize import deserialize

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('logfile')
    parser.add_argument('--stream', help='stream name with JPEG images', required=True)
    parser.add_argument('--stream2', help='2nd stream name with JPEG images')
    args = parser.parse_args()

    only_stream = lookup_stream_id(args.logfile, args.stream)
    if args.stream2 is not None:
        only_stream = [only_stream, lookup_stream_id(args.logfile, args.stream2)]
    peaks = {}
    with LogReader(args.logfile, only_stream_id=only_stream) as log:
        for timestamp, stream_id, data in log:
            buf = deserialize(data)
            img = cv2.imdecode(np.fromstring(buf, dtype=np.uint8), cv2.IMREAD_COLOR)
            s = skyline(img)
            peaks[stream_id] = find_peaks(s)
            img2 = draw_skyline(img, s)
            cv2.imshow('skyline' + str(stream_id), img2)
            KEY_Q = ord('q')
            KEY_SPACE = ord(' ')
            key = cv2.waitKey(1) & 0xFF
            if key == KEY_Q:
                break
            if key == KEY_SPACE:
                print(peaks)
                key = cv2.waitKey(0) & 0xFF
                if key == KEY_Q:
                    break


if __name__ == "__main__":
    main()

# vim: expandtab sw=4 ts=4
