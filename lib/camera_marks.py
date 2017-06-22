"""
  Camera landmarks detector

  test usage:
       ./camera_marks.py <jpg filename>|<image directory>

"""

import numpy as np
import cv2
import sys
import os


def openingClosing( binaryImg, ker = 4, method = "opening"):
    newBinaryImg = None
    kernel = np.ones( ( ker, ker ), np.uint8 )
    if method == "opening":
        newBinaryImg = cv2.morphologyEx( binaryImg, cv2.MORPH_OPEN, kernel)
    else: # closing
        newBinaryImg = cv2.morphologyEx( binaryImg, cv2.MORPH_CLOSE, kernel)
        
    return newBinaryImg


def marksDetektor( im ):
    b,g,r = cv2.split( im )
    b = b.astype(float)
    g = g.astype(float)
    r = r.astype(float)
    mat = b+g+r
    mat[mat == 0] = 1.0
    r[r < 20] = 0
    gray = r/mat*255
    gray = gray.astype(np.uint8)
    #cv2.imwrite( "test.png", gray )
    ret, binaryImg = cv2.threshold( gray, 110, 255,cv2.THRESH_BINARY)
    #cv2.imwrite( "test2.png", binaryImg )
    binaryImg = openingClosing( binaryImg )
    binaryImg = openingClosing( binaryImg, ker = 4, method = "closing" )
    #cv2.imwrite( "test3.png", binaryImg )
    contours, hierarchy= cv2.findContours( binaryImg, mode=cv2.RETR_EXTERNAL, method=cv2.CHAIN_APPROX_NONE)
    lMarks = []
    bRects = []
    areas = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        x,y,w,h = cv2.boundingRect(cnt)
        whRatio = w/float(h)
        arRatio = area/float(w*h)
        if area > 20 and arRatio > 0.5 and whRatio > 0.33 and whRatio < 3:
            lMarks.append(cnt)
            bRects.append([x,y,w,h])
            areas.append(area)
        
    return lMarks, bRects, areas


def draw_cnt(filename):
    print filename
    im  = cv2.imread(filename, 1)
    lMarks, bRects, areas = marksDetektor(im)
    conesXY, lMarksF = findCones(im)
    cv2.drawContours(im, lMarks, -1, (255,0,0), 2)
    cv2.drawContours(im, lMarksF, -1, (0,255,0), 2)
    cv2.imshow('cones', im)
    
def findCones(im):
    lMarks, bRects, areas = marksDetektor(im)
    conesXY = []
    lMarksF = []
    for ii in range(len(lMarks)):
        areaM = areas[ii]
        xM, yM, wM, hM = bRects[ii]
        xcM, ycM  = xM + wM/2.0, yM + hM/2.0
        cntAbove = 1
        for jj in range(len(lMarks)):
            areaO = areas[jj]
            xO, yO, wO, hO = bRects[jj]
            xcO, ycO  = xO + wO/2.0, yO + hO/2.0
            if abs(xcM-xcO) > wM/2.0:
                continue
            elif (ycM - ycO > 4*hM) or (ycM - ycO < hM):
                continue
            elif (areaO/areaM > 1.0) or (areaO/areaM < 0.1):
                continue
            cntAbove += 1
        #print cntAbove
        if cntAbove == 3:
            conesXY.append([xcM, ycM])
            lMarksF.append(lMarks[ii])
            
    return conesXY, lMarksF


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print __doc__
        sys.exit()

    if sys.argv[1].endswith('.jpg'):
        draw_cnt(sys.argv[1])
        cv2.waitKey()
    else:
        for filename in sorted(os.listdir(sys.argv[1])):
            if filename.endswith('jpg'):
                draw_cnt(os.path.join(sys.argv[1], filename))
                if cv2.waitKey(100) % 256 == 27:
                    break

# vim: expandtab sw=4 ts=4

