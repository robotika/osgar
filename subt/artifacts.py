"""
  Detect SubT Artifact in Camera Image
"""
from datetime import timedelta
import os
import tempfile
from io import StringIO

import cv2
import numpy as np

from osgar.node import Node
from osgar.bus import BusShutdownException


EXTINGUISHER = 'TYPE_EXTINGUISHER'
BACKPACK = 'TYPE_BACKPACK'
VALVE = 'TYPE_VALVE'
ELECTRICAL_BOX = 'TYPE_ELECTRICAL_BOX'
PHONE = 'TYPE_PHONE'
RADIO = 'TYPE_RADIO'
TOOLBOX = 'TYPE_TOOLBOX'
DUCT = 'TYPE_DUCT'
DRILL = 'TYPE_DRILL'
RESCUE_RANDY = 'TYPE_RESCUE_RANDY'
VENT = 'TYPE_VENT'
GAS = 'TYPE_GAS'


RED_THRESHOLD = 50  # virtual QVGA, used to be 100
YELLOW_THRESHOLD = 80
WHITE_THRESHOLD = 20000


def old_count_red(img):
    count = 0
    for x in range(320):
        for y in range(240):
            b, g, r = img[y][x]
            if r > 100 and r > 2 * g and r > 2 * b:
                count += 1
    return count


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


def count_red(img, filtered=False, stdout=None):
    b = img[:,:,0]
    g = img[:,:,1]
    r = img[:,:,2]
    mask = np.logical_and(r > 20, np.logical_and(r/2 > g, r/2 > b))  # QVGA virtual, dark images, used to be 100
    not_mask = np.logical_not(mask)
    kernel = np.ones((2,2), np.uint8)
    img2 = img.copy()
    img2[mask] = (255, 255, 255)
    img2[not_mask] = (0, 0, 0)
    img2 = cv2.erode(img2, kernel, iterations=1)
#    cv2.imwrite('artf.jpg', img2)
#        print('mask', mask.shape)
#        for x in range(mask.shape[0]):
#            for y in range(mask.shape[1]):
#                if mask[x][y]:
#                    print(x, y)
    b = img2[:,:,0]
    mask = b == 255
    if filtered:
        # detect largest blob in img2
        kernel = np.ones((3,3), np.uint8)
        img2 = cv2.dilate(img2, kernel, iterations=1)
#        cv2.imwrite('artf.jpg', img2)
        b = img2[:,:,0]
        mask = b == 255
        reddish = b.astype(np.uint8)
        __, bin_img = cv2.threshold(reddish, 160, 255, cv2.THRESH_BINARY)
        
        im2, contours, hierarchy = cv2.findContours(bin_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        best_cnt = None
        best_area = None
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if best_area is None or area > best_area:
                best_area = area
                best_cnt = cnt
#        print(best_area, best_cnt)
        x, y, w, h = cv2.boundingRect(best_cnt)

#        if stdout is not None:
#            # dump image to stdout
#            # https://stackoverflow.com/questions/50670326/how-to-check-if-point-is-placed-inside-contour
#            print(x, y, w, h)
#            for j in range(y, y + h):
#                s = ''
#                for i in range(x, x + w ):
#                    s += 'X' if mask[j][i] else '.'
#                stdout(s)
        # count, w, h, x_min, x_max
        return int(best_area), w, h, x, x+w

    return count_mask(mask)


def count_white(img):
    b = img[:,:,0]
    g = img[:,:,1]
    r = img[:,:,2]
    mask = np.logical_and(r >= 250, np.logical_and(g >= 250, b >= 250))
    return count_mask(mask)


def count_yellow(img):
    b = img[:,:,0]
    g = img[:,:,1]
    r = img[:,:,2]
    mask = np.logical_and(r >= 60, np.logical_and(r/4 > b, g/4 > b))
    return count_mask(mask)


def artf_in_scan(scan, width, img_x_min, img_x_max, verbose=False):
    """return precise artefact angle and distance for lidar & camera combination"""
    if scan is None:
        return 0, 0
    # the scan is already in mm, so angle is modified to int deg*100, ready to send
    x_min, x_max = img_x_min, img_x_max

    angular_resolution = len(scan) / 270
    mid_index = len(scan) // 2
    camera_fov_deg = 60
    deg_max = camera_fov_deg * (width / 2 - x_min) / width  # small value on the left corresponds to positive angle
    deg_min = camera_fov_deg * (width / 2 - x_max) / width
    tolerance = int(5 * angular_resolution)  # in paritular the valve is detected with offset
    left_index = mid_index + int(deg_min * angular_resolution) - tolerance
    right_index = mid_index + int(deg_max * angular_resolution) + tolerance
#    if verbose:
#        print('SubSelection', deg_min, deg_max, left_index, right_index, scan[left_index:right_index])

    tmp = [x if x > 0 else 100000 for x in scan]
    dist_mm = min(tmp[left_index:right_index])
    index = left_index + tmp[left_index:right_index].index(dist_mm)
    deg_100th = int(((index / angular_resolution) - 135) * 100)
    return deg_100th, dist_mm


class ArtifactDetector(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register("artf", "dropped", "debug_artf", "stdout")
        self.is_virtual = config['virtual_world']  # say different "dataset" (now red or all object detection)
        self.best = None
        self.best_count = 0
        self.best_img = None
        self.best_info = None
        self.best_scan = None
        self.verbose = False
        self.scan = None  # should laster initialize super()
        self.width = None  # detect from incoming images

    def waitForImage(self):
        channel = ""
        while channel != "image":
            self.time, channel, data = self.listen()
            setattr(self, channel, data)
        return self.time

    def stdout(self, *args, **kwargs):
        # maybe refactor to Node?
        output = StringIO()
        print(*args, file=output, **kwargs)
        contents = output.getvalue().strip()
        output.close()
        self.publish('stdout', contents)
        print(contents)

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
                self.detect(self.image)
        except BusShutdownException:
            pass

    def detect(self, image):
        img = cv2.imdecode(np.fromstring(image, dtype=np.uint8), 1)
        if self.width is None:
            self.stdout('Image resolution', img.shape)
            self.width = img.shape[1]
        assert self.width == img.shape[1], (self.width, img.shape[1])
        rcount, w, h, x_min, x_max = count_red(img)
        yellow_used = False
        if self.is_virtual and rcount < 20:
            ycount, w, h, x_min, x_max = count_yellow(img)
            if ycount > YELLOW_THRESHOLD:
                yellow_used = True
                count = ycount
            else:
                count = rcount
        else:
            count = rcount

        if self.verbose and count >= 20:
            print(self.time, img.shape, count, w, h, x_min, x_max, w/h, count/(w*h))
        if self.best_count > 0:
            self.best_count -= 1
        if count > RED_THRESHOLD:
            if self.best is None or count > self.best:
                self.best = count
                self.best_count = 10
                self.best_img = self.image
                self.best_info = w, h, x_min, x_max, (count == rcount), yellow_used  # RED used
                self.best_scan = self.scan

        if self.best is not None and self.best_count == 0:
            w, h, x_min, x_max, red_used, yellow_used = self.best_info
            if red_used:
                # revise noisy points
                img = cv2.imdecode(np.fromstring(self.best_img, dtype=np.uint8), 1)
                rcount, w, h, x_min, x_max = count_red(img, filtered=True, stdout=self.stdout)
                if rcount == 0:
                    self.stdout('Invalid after filtering! orig count=', self.best)
                    # reset detector
                    self.best = None
                    self.best_count = 0
                    self.best_img = None
                    self.best_info = None
                    self.best_scan = None
                    return
                self.stdout(rcount, w, h, x_min, x_max, w/h, count/(w*h))

            if red_used or yellow_used:
                deg_100th, dist_mm = artf_in_scan(self.best_scan, self.width, x_min, x_max, verbose=True)
            else:
                deg_100th, dist_mm = 0, 500  # in front of the robot

            if red_used:
                self.stdout('h/w', h/w)
#                if self.best < 1000:
#                    artf = DRILL  # VALVE - hack for simple02
#                elif h/w > 2.4:
#                    artf = EXTINGUISHER
#                elif h/w > 1.0:
#                    artf = BACKPACK
#                else:
#                    artf = TOOLBOX
#                if self.best < 1000:
#                    artf = DRILL  # VALVE - hack for simple02, fallback for empty image
#                if h/w > 2:
#                    artf = EXTINGUISHER
                artf = BACKPACK
            elif yellow_used:
                artf = RESCUE_RANDY  # used to be RADIO
            self.stdout(self.time, 'Relative position:', self.best, deg_100th, dist_mm, artf)

            dx_mm, dy_mm = 0, 0  # relative offset to current robot position
            # TODO if VALVE -> find it in scan
            self.publish('artf', [artf, deg_100th, dist_mm])
            self.publish('debug_artf', self.best_img)
#            filename = 'artf_%s_%d.jpg' % (artf, self.time.total_seconds())
#            with open(filename, 'wb') as f:
#                f.write(self.best_img)

            # reset detector
            self.best = None
            self.best_count = 0
            self.best_img = None
            self.best_info = None
            self.best_scan = None


class ArtifactReporter(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register("artf_cmd")
        self.path = config.get('path', 'call_base.txt')

    def update(self):  # hack, this method should be called run instead!
        channel = super().update()  # define self.time
        assert channel == "artf_xyz", channel

        print("DETECTED:")
        for art in self.artf_xyz:
            print(art)

        # TODO call SubT API
        # Make all reported information available atomically to avoid race
        # conditions between writing to and reading from the same file.
        dir = os.path.dirname(os.path.abspath(self.path))
        with tempfile.NamedTemporaryFile(mode="w", delete=False, dir=dir) as f:
            for artf_type, ix, iy, iz in self.artf_xyz:
                s = '%s %.2f %.2f %.2f\n' % (artf_type, ix/1000.0, iy/1000.0, iz/1000.0)
                f.write(s)
                self.publish('artf_cmd', bytes('artf ' + s, encoding='ascii'))
            f.close()
            os.rename(f.name, self.path)

        print('report completed')


if __name__ == '__main__':
    from unittest.mock import MagicMock
    from queue import Queue
    import argparse
    import datetime
    from osgar.bus import Bus

    parser = argparse.ArgumentParser(description='Run artifact detection and classification for given JPEG image')
    parser.add_argument('filename', help='JPEG filename')
    parser.add_argument('-v', '--verbose', help='verbose mode', action='store_true')
    args = parser.parse_args()

    with open(args.filename, 'rb') as f:
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
    tester.register('scan', 'image', 'tick')
    bus.connect('tester.scan', 'detector.scan')
    bus.connect('tester.image', 'detector.image')
    bus.connect('detector.artf', 'tester.artf')
    bus.connect('tester.tick', 'tester.tick')
    bus.connect('detector.dropped', 'tester.dropped')
    tester.publish('scan', [2000]*270)  # pretend that everything is at 2 meters
    detector.start()
    for i in range(10 + 1):  # workaround for local minima
        a = tester.listen()
#        print(i, a)
        tester.sleep(0.01)
        tester.publish('image', jpeg_data)
    detector.request_stop()
    detector.join()
    tester.publish('tick', None)
    a = tester.listen()
    print(a)

# vim: expandtab sw=4 ts=4
