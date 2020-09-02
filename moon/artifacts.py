"""
  Detect Cube Sat and Processing Plant artifacts
"""
from datetime import timedelta
import os
from io import StringIO
from statistics import median

import cv2
import numpy as np
from pathlib import Path

from osgar.node import Node
from osgar.bus import BusShutdownException
from moon.moonnode import CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_FOCAL_LENGTH

curdir = Path(__file__).parent

def union(a,b):
    x = min(a[0], b[0])
    y = min(a[1], b[1])
    w = max(a[0]+a[2], b[0]+b[2]) - x
    h = max(a[1]+a[3], b[1]+b[3]) - y
    return (x, y, w, h)

class ArtifactDetector(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register("artf", "dropped")
        self.verbose = False
        self.dump_dir = None  # optional debug ouput into directory
        self.scan = None  # should laster initialize super()
        self.depth = None  # more precise definiton of depth image
        self.width = None  # detect from incoming images
        self.look_for_artefacts = config.get('artefacts', [])
        self.estimate_distance = config.get('estimate_distance', False)

        window_size = 5
        min_disp = 16
        num_disp = 192-min_disp
        blockSize = window_size
        uniquenessRatio = 7
        speckleRange = 3
        speckleWindowSize = 75
        disp12MaxDiff = 200
        P1 = 8*3*window_size**2
        P2 = 32 * 3 * window_size ** 2
        self.stereo_calc = cv2.StereoSGBM_create(
            minDisparity = min_disp,
            numDisparities = num_disp,
            blockSize = window_size,
            uniquenessRatio = uniquenessRatio,
            speckleRange = speckleRange,
            speckleWindowSize = speckleWindowSize,
            disp12MaxDiff = disp12MaxDiff,
            P1 = P1,
            P2 = P2
        )
        self.Q = np.float32([[1, 0, 0, -0.5*CAMERA_WIDTH],
                             [0,-1, 0,  0.5*CAMERA_HEIGHT], # turn points 180 deg around x-axis,
                             [0, 0, 0,     CAMERA_FOCAL_LENGTH], # so that y-axis looks up
                             [0, 0, 1/0.42,      0]])

        self.detectors = [
            {
                'artefact_name': 'cubesat',
                'detector_type': 'classifier',
                'classifier': cv2.CascadeClassifier(str(curdir/'xml/cubesat.xml')),
                'min_size': 5,
                'max_size': 110,
                'subsequent_detects_required': 3
                },
            {
                'artefact_name': 'homebase',
                'detector_type': 'classifier',
                'classifier': cv2.CascadeClassifier(str(curdir/'xml/homebase.xml')),
                'min_size': 20,
                'max_size': 400,
                'subsequent_detects_required': 3
                },
            {
                'artefact_name': 'basemarker',
                'detector_type': 'colormatch',
                'min_size': 10,
                'max_size': 500,
                'mask': [CAMERA_HEIGHT//2,  CAMERA_HEIGHT, 0, CAMERA_WIDTH], # [Y,X] order, look only in lower half of the screen (avoid solar panels)
                'pixel_count_threshold': 100,
                'bbox_union_count': 1,
                'hue_max_difference': 10,
                'hue_match': 100, # from RGB 007DBD
                'subsequent_detects_required': 3  # noise will add some of this color, wait for a consistent sequence
            },
            {
                'artefact_name': 'homebase',
                'detector_type': 'colormatch',
                'min_size': 20,
                'max_size': 700,
                'mask': None,
                'pixel_count_threshold': 400,
                'bbox_union_count': 5,
                'hue_max_difference': 10,
                'hue_match': 19, # from RGB FFA616
                'subsequent_detects_required': 3
            },
            {
                'artefact_name': 'rover',
                'detector_type': 'colormatch',
                'min_size': 10,
                'max_size': 700,
                'mask': None,
                'pixel_count_threshold': 150,
                'bbox_union_count': 3,
                'hue_max_difference': 1,
                'hue_match': 26, # from RGB FFA616
                'subsequent_detects_required': 3
            },
            {
                'artefact_name': 'excavator_arm',
                'detector_type': 'colormatch',
                'min_size': 10,
                'max_size': 700,
                'mask': [0,  120, 0, CAMERA_WIDTH], # [Y,X] order
                'pixel_count_threshold': 150,
                'bbox_union_count': 3,
                'hue_max_difference': 1,
                'hue_match': 26, # from RGB FFA616
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
            elif channel == "right_image":
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
                    # this thread is always running but wait and drop images if simulation is slower
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

        limg = cv2.imdecode(np.frombuffer(left_image, dtype=np.uint8), cv2.IMREAD_COLOR)
        rimg = cv2.imdecode(np.frombuffer(right_image, dtype=np.uint8), cv2.IMREAD_COLOR)

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

        objects_detected = []
        for c in self.detectors:
            if c['artefact_name'] not in self.look_for_artefacts:
                continue

            if c['artefact_name'] not in self.detect_sequences:
                self.detect_sequences[c['artefact_name']] = 0

            if c['detector_type'] == 'colormatch':
                lower_hue = np.array([c['hue_match'] - c['hue_max_difference'],50,50])
                upper_hue = np.array([c['hue_match'] + c['hue_max_difference'],255,255])
                # Threshold the HSV image to get only the matching colors
                mask = cv2.inRange(hsv_blurred, lower_hue, upper_hue)
                if c['mask'] is not None:
                    m = np.zeros([CAMERA_HEIGHT,CAMERA_WIDTH], dtype=np.uint8)
                    m[c['mask'][0]:c['mask'][1],c['mask'][2]:c['mask'][3]] = 255
                    mask &= m

                bboxes = []
                contours = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                contours = contours[0] if len(contours) == 2 else contours[1]
                for cont in contours:
                    contours_poly = cv2.approxPolyDP(cont, 3, True)
                    x,y,w,h = cv2.boundingRect(contours_poly)
                    bboxes.append([int(x),int(y),int(w),int(h)])

                if len(bboxes) > 0:
                    sb = sorted(bboxes, key = box_area, reverse = True)[:c['bbox_union_count']]
                    bbox = sb[0]
                    for b in sb[1:]:
                        bbox = union(bbox,b)
                    x, y, w, h = bbox
                    match_count = cv2.countNonZero(mask[y:y+h,x:x+w])
                    if (
                            match_count > c['pixel_count_threshold'] and
                            w >= c['min_size'] and h >= c['min_size'] and
                            w <= c['max_size'] and h <= c['max_size']
                    ):
                        # print ("%s match count: %d; [%d %d %d %d]" % (c['artefact_name'], match_count, x, y, w, h))
                        objects_detected.append(c['artefact_name'])
                        if self.detect_sequences[c['artefact_name']] < c['subsequent_detects_required']:
                            # do not act until you have detections in a row
                            self.detect_sequences[c['artefact_name']] += 1
                        else:
                            if self.estimate_distance:
                                disp = self.stereo_calc.compute(limg_rgb, rimg_rgb).astype(np.float32) / 16.0
                                points = cv2.reprojectImageTo3D(disp, self.Q)
                                matching_points = points[mask != 0]
                                distances = matching_points[:,2] # third column are Z coords (distances)

                                mean = np.mean(distances)
                                sd = np.std(distances)
                                distances_clean = [x for x in distances if mean - 2 * sd < x < mean + 2 * sd]

                                #print("Artf distance: min %.1f median: %.1f" % (min(distances), median(distances)))
                                if len(distances_clean) == 0:
                                    distances_clean = distances
                                    # print("Artf cleaned: min %.1f median: %.1f" % (min(final_list), median(final_list)))
                                dist = max(0.0, min(distances_clean)) # subtract about half length of the rover
                            else:
                                dist = 0.0
                            results.append((c['artefact_name'], int(x), int(y), int(w), int(h), int(match_count), float(dist)))

            if c['detector_type'] == 'classifier':
                lfound = c['classifier'].detectMultiScale(limg_rgb, minSize =(c['min_size'], c['min_size']),  maxSize =(c['max_size'], c['max_size']))
                rfound = c['classifier'].detectMultiScale(rimg_rgb, minSize =(c['min_size'], c['min_size']),  maxSize =(c['max_size'], c['max_size']))

                if len(lfound) > 0 and len(rfound) > 0: # only report if both cameras see it
                    objects_detected.append(c['artefact_name'])
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

        for artefact_name in self.detect_sequences.keys():
            if artefact_name not in objects_detected:
                self.detect_sequences[artefact_name] = 0

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
