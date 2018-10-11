"""
TODO
"""

import sys
import cv2
import numpy as np


KERNEL = np.ones((10, 10), np.uint8)
yi, ye, xi, xe = 350, 550, 200, 1080


def showImg( img ):
    cv2.namedWindow('image',cv2.WINDOW_NORMAL)
    cv2.resizeWindow('image', 600,600)
    cv2.imshow( 'image', img )
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def ballDetect(img, ind):
    img2 = img[yi:ye, xi:xe, :]
    b,g,r = cv2.split(img2)
    b = b.astype(float)
    g = g.astype(float)
    r = r.astype(float)
    mat = b + g + r
    mat[mat == 0] = 1.0  # avoid division by 0
    r[r < 20] = 0        # ignore 'dark red'
    reddish = r/mat*255  # 'normalize' red, i.e. red compared to other color planes
    reddish = reddish.astype(np.uint8)
    #plt.hist( reddish.ravel(), 256,[0,256])
    #plt.show()
    
    #cv2.imwrite( "im/gray_%04d.png" % ind, reddish )
    __, binaryImg = cv2.threshold(reddish, 150, 255, cv2.THRESH_BINARY)
    
    binaryImg = cv2.morphologyEx(binaryImg, cv2.MORPH_OPEN, KERNEL)
    binaryImg = cv2.morphologyEx(binaryImg, cv2.MORPH_CLOSE, KERNEL)
    
    ___, contours, ___ = cv2.findContours( binaryImg, mode=cv2.RETR_EXTERNAL, method=cv2.CHAIN_APPROX_NONE)
    #cv2.drawContours(img2, contours, -1, (0,0,255), 3)
    #img[yi:ye,xi:xe, :] = img2
    #cv2.imwrite( "im/im_%04d.png" % ind, img )
    #cv2.imwrite( "im/bin_%04d.png" % ind, binaryImg )
    
    ret = []
    for cnt in contours:
        x,y,w,h = cv2.boundingRect(cnt)
        ret.append([x+xi, y+yi, w,h])
    
    return ret

if __name__ == "__main__":
    import argparse
    import matplotlib.pyplot as plt
    from osgar.logger import LogReader, lookup_stream_id
    from osgar.lib.serialize import deserialize

    parser = argparse.ArgumentParser(description='Extract features in laser scan')
    parser.add_argument('filename', help='input log file')
    parser.add_argument('--verbose', '-v', help="verbose mode", action='store_true')
    parser.add_argument('--draw', '-d', help="draw result", action='store_true')
    parser.add_argument('--index', '-i', help="scan index", type=int)
    args = parser.parse_args()

    filename = args.filename
    only_stream = lookup_stream_id(filename, 'camera.raw')
    index = args.index
    with LogReader(filename) as log:
        for ind, row in enumerate(log.read_gen(only_stream)):
            if index is not None and ind < index:
                continue
            timestamp, stream_id, data = row
            buf = deserialize(data)
            img = cv2.imdecode(np.fromstring(buf, dtype=np.uint8), 1)
            ballList = ballDetect(img, ind)
            print(ballList)
            
            if index is not None:
                if ind > 600:
                    break
                    
