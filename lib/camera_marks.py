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
    ret, binaryImg = cv2.threshold( gray, 100, 255,cv2.THRESH_BINARY)
    #cv2.imwrite( "test2.png", binaryImg )
    binaryImg = openingClosing( binaryImg )
    binaryImg = openingClosing( binaryImg, ker = 4, method = "closing" )
    #cv2.imwrite( "test3.png", binaryImg )
    contours, hierarchy= cv2.findContours( binaryImg, mode=cv2.RETR_EXTERNAL, method=cv2.CHAIN_APPROX_NONE)
    lMarks = []
    for cnt in contours:
        if cv2.contourArea(cnt) > 16:
            lMarks.append(cnt)
    
    return lMarks


def draw_cones(filename):
    print filename
    im  = cv2.imread(filename, 1)
    lMarks = marksDetektor(im)
    cv2.drawContours(im, lMarks, -1, (255,0,0), 2)
    cv2.imshow('cones', im)


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print __doc__
        sys.exit()

    if sys.argv[1].endswith('.jpg'):
        draw_cones(sys.argv[1])
        cv2.waitKey()
    else:
        for filename in os.listdir(sys.argv[1]):
            if filename.endswith('jpg'):
                draw_cones(os.path.join(sys.argv[1], filename))
                if cv2.waitKey(100) == 27:
                    break

# vim: expandtab sw=4 ts=4

