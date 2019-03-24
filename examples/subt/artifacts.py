"""
  Detect SubT Artifact in Camera Image
"""
from datetime import timedelta
import os
import tempfile
import pathlib

import cv2
import numpy as np

from osgar.node import Node


VIRTUAL_WORLD = False  # TODO more suitable parametrization, real robots now look only for red artifacts


EXTINGUISHER = 'TYPE_EXTINGUISHER'
BACKPACK = 'TYPE_BACKPACK'
VALVE = 'TYPE_VALVE'
ELECTRICAL_BOX = 'TYPE_ELECTRICAL_BOX'
PHONE = 'TYPE_PHONE'
RADIO = 'TYPE_RADIO'
TOOLBOX = 'TYPE_TOOLBOX'
DUCT = 'TYPE_DUCT'

RED_THRESHOLD = 100
YELLOW_THRESHOLD = 40
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


def count_red(img):
    b = img[:,:,0]
    g = img[:,:,1]
    r = img[:,:,2]
    mask = np.logical_and(r > 100, np.logical_and(r/2 > g, r/2 > b))
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
    mask = np.logical_and(r >= 240, np.logical_and(g >= 240, b < 180))
    return count_mask(mask)


def artf_in_scan(scan, img_x_min, img_x_max, verbose=False):
    """return precise artefact angle and distance for lidar & camera combination"""
    # the scan is already in mm, so angle is modified to int deg*100, ready to send
    x_min, x_max = img_x_min, img_x_max

    angular_resolution = len(scan) / 270
    mid_index = len(scan) // 2
    camera_fov_deg = 60
    deg_max = camera_fov_deg * (1280 / 2 - x_min) / 1280  # small value on the left corresponds to positive angle
    deg_min = camera_fov_deg * (1280 / 2 - x_max) / 1280
    tolerance = int(5 * angular_resolution)  # in paritular the valve is detected with offset
    left_index = mid_index + int(deg_min * angular_resolution) - tolerance
    right_index = mid_index + int(deg_max * angular_resolution) + tolerance
    if verbose:
        print('SubSelection', deg_min, deg_max, left_index, right_index, scan[left_index:right_index])

    tmp = [x if x > 0 else 100000 for x in scan]
    dist_mm = min(tmp[left_index:right_index])
    index = left_index + tmp[left_index:right_index].index(dist_mm)
    deg_100th = int(((index / angular_resolution) - 135) * 100)
    return deg_100th, dist_mm


class ArtifactDetector(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        self.best = None
        self.best_count = 0
        self.best_img = None
        self.best_info = None
        self.best_scan = None
        self.verbose = False
        self.scan = None  # should laster initialize super()

    def update(self):  # hack, this method should be called run instead!
        channel = super().update()  # define self.time
        if channel != "image":
            return channel

        # hack - test communication to BaseStation
#        if self.time > timedelta(minutes=4):
#            print('Published', self.best)
#            self.publish('artf', EXTINGUISHER)
#        return channel
        # END OF HACK ....

        img = cv2.imdecode(np.fromstring(self.image, dtype=np.uint8), 1)
        rcount, w, h, x_min, x_max = count_red(img)
        yellow_used = False
        if VIRTUAL_WORLD and rcount == 0:
            wcount, w, h, x_min, x_max = count_white(img)
            if wcount > WHITE_THRESHOLD and 1.8 < w/h < 1.9 and wcount/(w*h) > 0.6:
                count = wcount
            else:
                ycount, w, h, x_min, x_max = count_yellow(img)
                if ycount > YELLOW_THRESHOLD:
                    yellow_used = True
                    count = ycount
                else:
                    count = 0
        else:
            count = rcount

        if self.verbose and count > 0:
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
            print('Published', self.best)
            if red_used or yellow_used:
                deg_100th, dist_mm = artf_in_scan(self.best_scan, x_min, x_max, verbose=True)
            else:
                deg_100th, dist_mm = 0, 500  # in front of the robot
            print('Relative position:', deg_100th, dist_mm)

            if red_used:
                print('h/w', h/w)
                if self.best < 1000:
                    artf = VALVE
                elif h/w > 2.4:
                    artf = EXTINGUISHER
                elif h/w > 1.0:
                    artf = BACKPACK
                else:
                    artf = TOOLBOX
            elif yellow_used:
                artf = RADIO
            else:
                artf = ELECTRICAL_BOX

            dx_mm, dy_mm = 0, 0  # relative offset to current robot position
            # TODO if VALVE -> find it in scan
            self.publish('artf', [artf, deg_100th, dist_mm])
            filename = 'artf_%s_%d.jpg' % (artf, self.time.total_seconds())
            with open(filename, 'wb') as f:
                f.write(self.best_img)

            # reset detector
            self.best = None
            self.best_count = 0
            self.best_img = None
            self.best_info = None
            self.best_scan = None
        return channel


class ArtifactReporter(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
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
        dir = pathlib.Path(self.path).resolve().parent
        with tempfile.NamedTemporaryFile(mode="w", delete=False, dir=dir) as f:
            for artf_type, ix, iy, iz in self.artf_xyz:
                f.write('%s %.2f %.2f %.2f\n' % (artf_type, ix/1000.0, iy/1000.0, iz/1000.0))
            f.close()
            os.rename(f.name, self.path)

        print('report completed')


if __name__ == '__main__':
    from unittest.mock import MagicMock
    from queue import Queue
    import argparse
    from osgar.bus import BusHandler

    parser = argparse.ArgumentParser(description='Run artifact detection and classification for given JPEG image')
    parser.add_argument('filename', help='JPEG filename')
    parser.add_argument('-v', '--verbose', help='verbose mode', action='store_true')
    args = parser.parse_args()

    with open(args.filename, 'rb') as f:
        jpeg_data = f.read()

    config = {}
    logger = MagicMock()
    logger.register = MagicMock(return_value=1)
    output = Queue()
    bus = BusHandler(logger=logger, out={'artf': [(output, 'artf')]})
    detector = ArtifactDetector(config, bus)
    detector.verbose = args.verbose
    detector.start()
    bus.queue.put((timedelta(0), 'scan', [2000]*270))  # pretend that everything is at 2 meters
    for i in range(10 + 1):  # workaround for local minima
        bus.queue.put((timedelta(0), 'image', jpeg_data))
    detector.request_stop()
    detector.join()
    if output.empty():
        print("NO OUTPUT")
    else:
        print(output.get()[2])  # print only output data

# vim: expandtab sw=4 ts=4
