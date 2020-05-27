"""
  Detect Cube Sat and Processing Plant artifacts
"""
from datetime import timedelta
import os
from io import StringIO

import cv2
import numpy as np

from osgar.node import Node
from osgar.bus import BusShutdownException

g_mask = None
MIN_CYAN_COUNT = 300

def count_mask(mask):
    """Count statistics and bounding box for given image mask"""
    count = int(mask.sum())
    if count == 0:
        return count, None, None, None, None

    # argmax for mask finds the first True value
    x_min = (mask.argmax(axis=0) != 0).argmax()
    x_max = mask.shape[1] - np.flip((mask.argmax(axis=0) != 0), axis=0).argmax() - 1
    w = (mask.shape[1] - np.flip((mask.argmax(axis=0) != 0), axis=0).argmax()
            - (mask.argmax(axis=0) != 0).argmax())
    h = (mask.shape[0] - np.flip((mask.argmax(axis=1) != 0), axis=0).argmax()
            - (mask.argmax(axis=1) != 0).argmax())
    return count, w, h, x_min, x_max

def count_cyan(img, filtered=False, stdout=None):
    # NASA SRC logo color is #007DBD or (0,125,189)
    # assume valid colors are between that and lighter #E5F2F8 (229, 242, 248) (given the illumination)
    # valid rgb: ratios for r,g,b must be similar, ie for any given pixel, each r,g,b must in the same place between both end rgb values
    
#    r ratio = (r - 0) / (229 - 0)
#    g ratio = (g - 125) / (242 - 125)
#    b ratio = (b - 189) / (248 - 189)
    
    b = img[:,:,0]
    g = img[:,:,1]
    r = img[:,:,2]
    
    mask = np.logical_and((r - 0) / (229 - 0) < 1, np.logical_and(abs(((r - 0) / (229 - 0)) - ((g - 125) / (242 - 125))) < 0.2 , abs(((b - 189) / (248 - 189)) - ((g - 125) / (242 - 125))) < 0.2))
    not_mask = np.logical_not(mask)
    img2 = img.copy()
    img2[mask] = (255, 255, 255)
    img2[not_mask] = (0, 0, 0)

    global g_mask
    g_mask = mask.copy()
    return count_mask(mask)


class ArtifactDetector(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register("artf", "dropped", "debug_artf")
        self.best = None
        self.best_count = 0
        self.best_img = None
        self.best_info = None
        self.best_scan = None
        self.best_depth = None
        self.verbose = False
        self.dump_dir = None  # optional debug ouput into directory
        self.scan = None  # should laster initialize super()
        self.depth = None  # more precise definiton of depth image
        self.width = None  # detect from incoming images
        self.cascade = [
            {
                'artefact_name': 'cubesat',
                'classifier': cv2.CascadeClassifier('/osgar/moon/xml/cubesat.xml'),
                'min_size': 5,
                'max_size': 110,
                'subsequent_detects': 0
                },
            {
                'artefact_name': 'homebase',
                'classifier': cv2.CascadeClassifier('/osgar/moon/xml/homebase.xml'),
                'min_size': 50,
                'max_size': 300,
                'subsequent_detects': 0
            }
        ]

    def stdout(self, *args, **kwargs):
        # maybe refactor to Node?
        output = StringIO()
        print(*args, file=output, **kwargs)
        contents = output.getvalue().strip()
        output.close()
#        self.publish('stdout', contents)
        print(contents)

    def waitForImage(self):
        self.left_image = self.right_image = None
        while self.left_image is None or self.right_image is None:  
            self.time, channel, data = self.listen()
            if channel == "left_image":
                self.left_image = data
            else:
                self.right_image = data
        return self.time

    def run(self):
        try:
            dropped = 0
            while True:
                now = self.publish("dropped", dropped)
                dropped = -1
                timestamp = now
                while timestamp <= now:
                    timestamp = self.waitForImage()
                    dropped += 1
                self.detect(self.left_image, self.right_image)
        except BusShutdownException:
            pass

    def detect(self, left_image, right_image):
        limg = cv2.imdecode(np.fromstring(left_image, dtype=np.uint8), 1)
        rimg = cv2.imdecode(np.fromstring(right_image, dtype=np.uint8), 1)


        if self.width is None:
            self.stdout('Image resolution', limg.shape)
            self.width = limg.shape[1]
        assert self.width == limg.shape[1], (self.width, limg.shape[1])


        ccount, w, h, x_min, x_max = count_cyan(limg[0:480,300:340])
        if ccount > MIN_CYAN_COUNT: 
            self.publish('artf', ['basemarker', 300, 0, 60, 480, ccount])
        
        limg_rgb = cv2.cvtColor(limg, cv2.COLOR_BGR2RGB) 
        rimg_rgb = cv2.cvtColor(rimg, cv2.COLOR_BGR2RGB) 

        for c in self.cascade:
            lfound = c['classifier'].detectMultiScale(limg_rgb, minSize =(c['min_size'], c['min_size']),  maxSize =(c['max_size'], c['max_size'])) 
            rfound = c['classifier'].detectMultiScale(rimg_rgb, minSize =(c['min_size'], c['min_size']),  maxSize =(c['max_size'], c['max_size'])) 

            if len(lfound) > 0 and len(rfound) > 0: # only report if both cameras see it
                if c['subsequent_detects'] < 3: # do not act until you have at least 3 detections in a row
                    c['subsequent_detects'] += 1
                else:

                    # TODO: tweak the filtering (blur and threshold), sometimes not all background is filtered out and the bbox looks bigger than it should be
                    x,y,width,height = lfound[0]
#                    print(self.time, "Pre: %d %d %d %d" % (x,y,width,height))
                    gray = cv2.cvtColor(limg_rgb[y:y+height, x:x+width], cv2.COLOR_BGR2GRAY)
                    blur = cv2.medianBlur(gray,3) # some frames have noise, need to blur otherwise threshold doesn't work
                    th, threshed = cv2.threshold(blur, 30, 255, cv2.THRESH_BINARY)
                    coords = cv2.findNonZero(threshed)
                    nonzerocount = cv2.countNonZero(threshed)
                    nx, ny, nw, nh = cv2.boundingRect(coords)
#                    print(self.time, "Post: %d %d %d %d" % (x+nx,y+ny,nw,nh))

                    self.publish('artf', [c['artefact_name'], int(x+nx), int(y+ny), int(nw), int(nh), int(nonzerocount)])
                        
            else:
                c['subsequent_detects'] = 0

def debug2dir(filename, out_dir):
    from osgar.logger import LogReader, lookup_stream_names
    from osgar.lib.serialize import deserialize

    names = lookup_stream_names(filename)
    assert 'detector.debug_artf' in names, names
    assert 'detector.artf' in names, names
    assert 'rosmsg.sim_time_sec' in names, names
    image_id = names.index('detector.debug_artf') + 1
    artf_id = names.index('detector.artf') + 1
    sim_sec_id = names.index('rosmsg.sim_time_sec') + 1
    sim_time_sec = None
    image = None
    artf = None
    for dt, channel, data in LogReader(filename, only_stream_id=[image_id, artf_id, sim_sec_id]):
        data = deserialize(data)
        if channel == sim_sec_id:
            sim_time_sec = data
        elif channel == image_id:
            image = data
            assert artf is not None
            time_sec = sim_time_sec if sim_time_sec is not None else int(dt.total_seconds())
            name = os.path.basename(filename)[:-4] + '-' + artf[0] + '-' + str(time_sec) + '.jpg'
            print(name)
            with open(os.path.join(out_dir, name), 'wb') as f:
                f.write(image)
        elif channel == artf_id:
            artf = data


if __name__ == '__main__':
    from unittest.mock import MagicMock
    from queue import Queue
    import argparse
    import datetime
    import sys
    from osgar.bus import Bus

    parser = argparse.ArgumentParser(description='Run artifact detection and classification for given JPEG image')
    parser.add_argument('filename', help='JPEG filename')
    parser.add_argument('--debug2dir', help='dump clasified debug images into directory')
    parser.add_argument('-v', '--verbose', help='verbose mode', action='store_true')
    args = parser.parse_args()

    if args.debug2dir is not None:
        debug2dir(args.filename, args.debug2dir)
        sys.exit()

    with open(args.filename.replace('.npz', '.jpg'), 'rb') as f:
        jpeg_data = f.read()

    config = {'virtual_world': True}  # for now
    logger = MagicMock()
    logger.register = MagicMock(return_value=1)
    def counter():
        start = datetime.datetime.utcnow()
        while True:
            dt = datetime.datetime.utcnow() - start
            yield dt
    logger.write = MagicMock(side_effect=counter())
    bus = Bus(logger)
    detector = ArtifactDetector(config, bus.handle('detector'))
    detector.verbose = args.verbose
    tester = bus.handle('tester')
    tester.register('scan', 'left_image', 'right_image', 'tick')
    bus.connect('tester.scan', 'detector.scan')
    bus.connect('tester.left_image', 'detector.left_image')
    bus.connect('tester.right_image', 'detector.right_image')
    bus.connect('detector.artf', 'tester.artf')
    bus.connect('tester.tick', 'tester.tick')
    bus.connect('detector.dropped', 'tester.dropped')
    tester.publish('scan', [2000]*270)  # pretend that everything is at 2 meters
    detector.start()
    for i in range(10 + 1):  # workaround for local minima
        a = tester.listen()
#        print(i, a)
        tester.sleep(0.01)
        tester.publish('left_image', jpeg_data)  # TODO right image
    detector.request_stop()
    detector.join()
    tester.publish('tick', None)
    a = tester.listen()
    print(a)

# vim: expandtab sw=4 ts=4
