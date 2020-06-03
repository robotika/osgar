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


class ArtifactDetector(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register("artf", "dropped")
        self.verbose = False
        self.dump_dir = None  # optional debug ouput into directory
        self.scan = None  # should laster initialize super()
        self.depth = None  # more precise definiton of depth image
        self.width = None  # detect from incoming images
        self.detectors = [
            {
                'artefact_name': 'cubesat',
                'detector_type': 'classifier',
                'classifier': cv2.CascadeClassifier('/osgar/moon/xml/cubesat.xml'),
                'min_size': 5,
                'max_size': 110,
                'subsequent_detects_required': 3
                },
            {
                'artefact_name': 'basemarker',
                'detector_type': 'colormatch',
                'mser': cv2.MSER_create(_min_area=100),
                'min_size': 50,
                'max_size': 500,
                'pixel_count_threshold': 100,
                'hue_max_difference': 10,
                'hue_match': 100, # from RGB 007DBD
                'subsequent_detects_required': 3  # noise will add some of this color, wait for a consistent sequence
            },
            {
                'artefact_name': 'homebase',
                'detector_type': 'colormatch',
                'mser': cv2.MSER_create(_min_area=400),
                'min_size': 20,
                'max_size': 700,
                'pixel_count_threshold': 400,
                'hue_max_difference': 10,
                'hue_match': 19, # from RGB FFA616
                'subsequent_detects_required': 3
            }
        ]
        self.detect_sequences = {}
        
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
                self.detect_and_publish(self.left_image, self.right_image)
        except BusShutdownException:
            pass

    def detect_and_publish(self, left_image, right_image):
        results = self.detect(left_image, right_image)
        for r in results:
            self.publish('artf', r)

    def detect(self, left_image, right_image):
        results = []

        limg = cv2.imdecode(np.frombuffer(left_image, dtype=np.uint8), 1)
        rimg = cv2.imdecode(np.frombuffer(right_image, dtype=np.uint8), 1)

        if self.width is None:
            self.stdout('Image resolution', limg.shape)
            self.width = limg.shape[1]
        assert self.width == limg.shape[1], (self.width, limg.shape[1])


        def box_area(b):
            return b[2]*b[3]
        
        limg_rgb = cv2.cvtColor(limg, cv2.COLOR_BGR2RGB) 
        rimg_rgb = cv2.cvtColor(rimg, cv2.COLOR_BGR2RGB) 
        hsv = cv2.cvtColor(limg, cv2.COLOR_BGR2HSV)
        hsv_blurred = cv2.medianBlur(hsv,5) # some frames have noise, need to blur otherwise threshold doesn't work

        for c in self.detectors:
            if c['artefact_name'] not in self.detect_sequences:
                self.detect_sequences[c['artefact_name']] = 0

            if c['detector_type'] == 'colormatch':
                lower_hue = np.array([c['hue_match'] - c['hue_max_difference'],50,50])
                upper_hue = np.array([c['hue_match'] + c['hue_max_difference'],255,255])
                # Threshold the HSV image to get only the matching colors
                mask = cv2.inRange(hsv_blurred, lower_hue, upper_hue)

                _, bboxes = c['mser'].detectRegions(mask)
                if len(bboxes) > 0:
                    sb = sorted(bboxes, key = box_area, reverse = True)[:1]
                    x, y, w, h = sb[0]
                    match_count = cv2.countNonZero(mask[y:y+h,x:x+w])
                    if (
                            match_count > c['pixel_count_threshold'] and
                            w >= c['min_size'] and h >= c['min_size'] and
                            w <= c['max_size'] and h <= c['max_size']
                    ):
                        # print ("%s match count: %d; [%d %d %d %d]" % (c['artefact_name'], match_count, x, y, w, h))
                        if self.detect_sequences[c['artefact_name']] < c['subsequent_detects_required']:
                            # do not act until you have detections in a row
                            self.detect_sequences[c['artefact_name']] += 1
                        else:
                            results.append((c['artefact_name'], int(x), int(y), int(w), int(h), int(match_count)))
                    else:
                        self.detect_sequences[c['artefact_name']] = 0
                else:
                    self.detect_sequences[c['artefact_name']] = 0
                    

            if c['detector_type'] == 'classifier':
                lfound = c['classifier'].detectMultiScale(limg_rgb, minSize =(c['min_size'], c['min_size']),  maxSize =(c['max_size'], c['max_size'])) 
                rfound = c['classifier'].detectMultiScale(rimg_rgb, minSize =(c['min_size'], c['min_size']),  maxSize =(c['max_size'], c['max_size'])) 

                if len(lfound) > 0 and len(rfound) > 0: # only report if both cameras see it
                    if self.detect_sequences[c['artefact_name']] < c['subsequent_detects_required']: # do not act until you have detections in a row
                        self.detect_sequences[c['artefact_name']] += 1
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

                        results.append((c['artefact_name'], int(x+nx), int(y+ny), int(nw), int(nh), int(nonzerocount)))
                else:
                    self.detect_sequences[c['artefact_name']] = 0

        return results


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
