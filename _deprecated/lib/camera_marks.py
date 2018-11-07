"""
  Camera landmarks detector

  test usage:
       ./camera_marks.py <jpg filename>|<image directory>

"""

import numpy as np
import cv2
import sys
import os
import zipfile


KERNEL = np.ones((4, 4), np.uint8)


def get_red_areas_contours(image):
    """Find contours corresponding to red/orange cone pixels"""

    # binarize RGB image by "reddish" (orange + red) pixels
    b,g,r = cv2.split(image)
    b = b.astype(float)
    g = g.astype(float)
    r = r.astype(float)
    mat = b + g + r
    mat[mat == 0] = 1.0  # avoid division by 0
    r[r < 20] = 0        # ignore 'dark red'
    reddish = r/mat*255  # 'normalize' red, i.e. red compared to other color planes
    reddish = reddish.astype(np.uint8)
    __, binaryImg = cv2.threshold(reddish, 110, 255, cv2.THRESH_BINARY)
    binaryImg = cv2.morphologyEx(binaryImg, cv2.MORPH_OPEN, KERNEL)
    binaryImg = cv2.morphologyEx(binaryImg, cv2.MORPH_CLOSE, KERNEL)

    __, contours, hierarchy= cv2.findContours(binaryImg, mode=cv2.RETR_EXTERNAL, method=cv2.CHAIN_APPROX_NONE)
    lMarks = []
    bRects = []
    areas = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        x,y,w,h = cv2.boundingRect(cnt)
        whRatio = w/float(h)
        arRatio = area/float(w*h)
        if area > 20 and arRatio > 0.5 and 0.33 < whRatio < 3:
            lMarks.append(cnt)
            bRects.append([x,y,w,h])
            areas.append(area)
        
    return lMarks, bRects, areas


def find_cone_parts(rects, areas):
    """Find 3 cone parts and return them as array of triplets"""
    assert len(rects) == len(areas), (len(rects), len(areas))
    triplets = []
    for ii in range(len(rects)):
        areaM = areas[ii]
        xM, yM, wM, hM = rects[ii]
        xcM, ycM  = xM + wM/2.0, yM + hM/2.0
        triplet = [ii]
        for jj in range(len(rects)):
            areaO = areas[jj]
            xO, yO, wO, hO = rects[jj]
            xcO, ycO  = xO + wO/2.0, yO + hO/2.0
            if (abs(xcM-xcO) <= wM/2.0 and   # in the same vertical position
                hM <= ycM - ycO <= 4*hM and  # not far away
                0.1 <= areaO/areaM <= 1.0):  # decreasing area size
                triplet.append(jj)
        if len(triplet) == 3:
            triplets.append(tuple(triplet))
            
    return triplets


def find_cones(image):
    """Find cone described with 3 red 'pyramidal' contours"""

    __, rects, areas = get_red_areas_contours(image)
    triplets = find_cone_parts(rects, areas)
    
    # compute bounding box of all 3 parts
    cones = []
    for triplet in triplets:
        xx, yy, xX, yY = [], [], [], []
        for i in triplet:
            x, y, w, h = rects[i]
            xx.append(x)
            yy.append(y)
            xX.append(x + w)
            yY.append(y + h)
        x, y, w, h = min(xx), min(yy), max(xX)-min(xx), max(yY)-min(yy)
        cones.append((x, y, w, h))

    return cones


def draw_cnt(filename):
    print(filename)
    if '.zip' in filename:
        zipname, filename = os.path.split(filename)
        buf = np.fromstring(zipfile.ZipFile(zipname).read(filename), dtype=np.uint8)
        im = cv2.imdecode(buf, -1)  # OpenCV3: cv2.CV_LOAD_IMAGE_COLOR no longer supported??
    else:
        im  = cv2.imread(filename, 1)

    lMarks, bRects, areas = get_red_areas_contours(im)
    cones = find_cones(im)
    print(cones)
#    cv2.drawContours(im, lMarks, -1, (255,0,0), 2)
    
    for x, y, w, h in cones:
        cv2.rectangle(im, (x, y), (x+w, y+h), (0, 255, 0), 2)
    cv2.imshow('cones', im)
    return cones


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit()

    if sys.argv[1].endswith('.jpg'):
        draw_cnt(sys.argv[1])
        cv2.waitKey()
    else:
        if sys.argv[1].endswith('.zip'):
            names = zipfile.ZipFile(sys.argv[1]).namelist()
        else:
            names = os.listdir(sys.argv[1])
        for filename in sorted(names):
            if filename.endswith('jpg') and not filename.startswith('camono'):
                # ignore monochromatic images
                conesXY = draw_cnt(os.path.join(sys.argv[1], filename))
                wait_time = 100 if len(conesXY)==0 else 1000
                if cv2.waitKey(wait_time) % 256 == 27:
                    break

# vim: expandtab sw=4 ts=4

