import cv2
import numpy as np
from matplotlib import pyplot as plt

#img = cv2.imread('messi5.jpg',0)
img = cv2.imread(r'm:\git\subt-artf\virtual\vent\vent1.jpg',0)
img2 = img.copy()
#template = cv2.imread('template.jpg',0)
template = cv2.imread(r'm:\git\subt-artf\virtual\vent\template.jpg',0)
w, h = template.shape[::-1]

# All the 6 methods for comparison in a list
methods = ['cv2.TM_CCOEFF', 'cv2.TM_CCOEFF_NORMED', 'cv2.TM_CCORR',
            'cv2.TM_CCORR_NORMED', 'cv2.TM_SQDIFF', 'cv2.TM_SQDIFF_NORMED']

methods = ['cv2.TM_CCORR_NORMED']


def match_template(img):
#    img = img2.copy()

    # Apply template Matching
    res = cv2.matchTemplate(img, template, cv2.TM_CCORR_NORMED)
    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
    print(max_val, max_loc)
    top_left = max_loc
    bottom_right = (top_left[0] + w, top_left[1] + h)
    return max_val


def draw():
    cv2.rectangle(img,top_left, bottom_right, 255, 2)

    plt.subplot(121),plt.imshow(res,cmap = 'gray')
    plt.title('Matching Result'), plt.xticks([]), plt.yticks([])
    plt.subplot(122),plt.imshow(img,cmap = 'gray')
    plt.title('Detected Point'), plt.xticks([]), plt.yticks([])
    plt.suptitle(meth)

    plt.show()


if __name__ == '__main__':
    import argparse
    from osgar.logger import LogReader, lookup_stream_id
    from osgar.lib.serialize import deserialize

    parser = argparse.ArgumentParser(description='Run artifact detection and classification for given JPEG image')
    parser.add_argument('filename', help='logfile')
    args = parser.parse_args()

    filename = args.filename

    stream_id = lookup_stream_id(filename, 'rosmsg.image')
    for dt, channel, data in LogReader(filename, only_stream_id=stream_id):
        buf = deserialize(data)
        img = cv2.imdecode(np.fromstring(buf, dtype=np.uint8), 0)
        
        val = match_template(img)
        assert val < 0.98, dt




# vim: expandtab sw=4 ts=4

