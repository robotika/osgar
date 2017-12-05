"""
  Simple drawing tool for image annotations

  test usage:
       python ./annot.py <jpg filename>|<image directory> >> tmp.txt

"""

import numpy as np
import cv2
import sys
import os
import zipfile


def draw_cones(image, cones):
    im = image.copy()
    for x, y, w, h in cones:
        cv2.rectangle(im, (x, y), (x+w, y+h), (0, 255, 0), 2)
    cv2.imshow('cones', im)


def onmouse(event, x, y, flags, param):
    image, cones = param
    if event == cv2.EVENT_LBUTTONDOWN:
        cones.append( (x, y, 0, 0))
    elif event == cv2.EVENT_LBUTTONUP:
        assert len(cones) > 0
        assert cones[-1][2] == 0, cones[-1]
        assert cones[-1][3] == 0, cones[-1]
        x0, y0, _, _ = cones[-1]
        cones[-1] = (x0, y0, x-x0, y-y0)
        draw_cones(image, cones)


def annot_cones(filename):
    if '.zip' in filename:
        zipname, filename = os.path.split(filename)
        buf = np.fromstring(zipfile.ZipFile(zipname).read(filename), dtype=np.uint8)
        im = cv2.imdecode(buf, cv2.CV_LOAD_IMAGE_COLOR)
    else:
        im  = cv2.imread(filename, 1)

    cones = []
    draw_cones(im, cones)
    cv2.setMouseCallback('cones', onmouse, (im, cones))
    while True:
        key = cv2.waitKey() % 256
        if key == 27:  # ESC - termination
            return None
        elif key == ord(' '):  # SPACE - skip image
            return filename, None
        elif key == 8:  # Backspace - remove last detection
            if len(cones) > 0:
                cones.pop()
                draw_cones(im, cones)
        else:
            break
    return filename, cones


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit()

    if sys.argv[1].endswith('.jpg'):
        print((annot_cones(sys.argv[1])))
    else:
        if sys.argv[1].endswith('.zip'):
            names = zipfile.ZipFile(sys.argv[1]).namelist()
        else:
            names = os.listdir(sys.argv[1])
        for filename in sorted(names):
            if filename.endswith('jpg') and not filename.startswith('camono'):
                # ignore monochromatic images
                cones = annot_cones(os.path.join(sys.argv[1], filename))
                if cones is None:
                    break
                print(cones) 

# vim: expandtab sw=4 ts=4

