#!/usr/bin/python
"""
   Play all images in given directory
   usage:
       ./play.py <dir>
"""

# original very old demhist.py hacked by m3d

import sys
import os
import cv2
import cv2.cv as cv

_brightness = 0
Gbrightness = 0

src_image=None
dst_image=None

# brightness/contrast callback function
def update_brightness( val ):
    global Gbrightness    # global tag is required, or we get UnboundLocalError
    Gbrightness = val
    update_brightcont( )


def update_brightcont():
    # no global tag required for images ???
    brightness = Gbrightness;
    dst_image = cv2.imread( files[brightness] );
    print files[brightness]
    cv2.imshow( "image", dst_image );


if __name__ == "__main__":
  # collect all .jpg files in directory
    if len(sys.argv) < 2:
      print __doc__
      sys.exit(1)
    dir = sys.argv[1]    
    names = os.listdir( dir )
    files = []
    for filename in names:
      if ".jpg" in filename:
        files.append( dir + "/" + filename )

    src_image = cv2.imread( files[0] );

    if src_image == None:
        print "Image was not loaded.";
        sys.exit(-1)

    dst_image = src_image

    cv.NamedWindow("image", 0);
    cv.CreateTrackbar("index", "image", _brightness, len(files) - 1, update_brightness);

    update_brightcont();
    key = cv2.waitKey(0)
    while key in [ord('P'), ord('p')]:
      for i in range(Gbrightness, len(files)):
        update_brightness(i)
        key = cv2.waitKey(100)
        if key in [27] + [ord(x) for x in ['q','Q','p','P']]:
          break
      if key in [ord('P'), ord('p'), -1]:
        # pause or reached End of images
        update_brightcont();
        key = cv2.waitKey(0)

