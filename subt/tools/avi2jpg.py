"""
   extract JPEG images from AVI video
   usage:
      python3 avi2jpg.py <video.avi> <output dir>
"""
import os.path

import cv2


def avi2jpg(filename, out_dir, start_index=0, end_index=None, show=True):
    cap = cv2.VideoCapture(filename)
    index = -1
    while(cap.isOpened()):
        ret, frame = cap.read()
        if frame is None:
            break
        index += 1
        if index < start_index:
            continue
        if end_index is not None and index >= end_index:
            break

        out_name = os.path.join(out_dir, 'img_%04d.jpg' % index)
        cv2.imwrite(out_name, frame)
        if show:
            cv2.imshow('image', frame)
            if cv2.waitKey(50) == 27:                
                break
        
    return index


if __name__ == "__main__":
    import sys
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(-1)

    print(avi2jpg(sys.argv[1], sys.argv[2]))

# vim: expandtab sw=4 ts=4
