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
HELMET = 'TYPE_HELMET'
ROPE = 'TYPE_ROPE'

RED_THRESHOLD = 100 #300  # for close backpacks was good 1000  # virtual QVGA=50, used to be 100, urban=1000
YELLOW_THRESHOLD = 200  #500  # was 80
YELLOW_MAX_THRESHOLD = 4000  # 2633 known example
RED_YELLOW_MIN_3D_THRESHOLD = 50  # number of colored pixels in given depth distance threshold
WHITE_THRESHOLD = 20000

CO2_REPORT_LIMIT = 800
PHONE_PREFIX = "PhoneArtifact"
PHONE_SIGNAL_LIMIT = -50


g_mask = None

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
    mask = np.logical_and(r > 50, np.logical_and(r/3 > g, r/3 > b))  # QVGA virtual, dark images, used to be 100, r/2
                                                                     # 3/4-VGA urban, light, strictly red r/3
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
    global g_mask
    g_mask = mask.copy()
    if filtered:
        # detect largest blob in img2
        kernel = np.ones((3,3), np.uint8)
        img2 = cv2.dilate(img2, kernel, iterations=1)
#        cv2.imwrite('artf.jpg', img2)
        b = img2[:,:,0]
        mask = b == 255
        reddish = b.astype(np.uint8)
        __, bin_img = cv2.threshold(reddish, 160, 255, cv2.THRESH_BINARY)

        # cv2.findCountours for opencv == 3 returns 3 values while for opencv == 4 returns just 2
        contours, hierarchy = cv2.findContours(bin_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2:]
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
    mask = np.logical_and(np.logical_and(r >= 60, g*0.95 > r), np.logical_and(r*0.5 > b, g*0.5 > b))
    global g_mask
    g_mask = mask.copy()
    # debug
#    img2 = img.copy()
#    img2[mask] = (0, 0, 255)
#    cv2.imwrite('artf.jpg', img2)
    return count_mask(mask)


def count_orange_blue(img):  # for phone
    b = img[:,:,0]
    g = img[:,:,1]
    r = img[:,:,2]
    mask_blue = np.logical_and(b > 40, np.logical_and(b/1.5 > g, b/2 > r))

    # detect phone only in the lowest 1/4 of the image (the phone is lying on the ground)
    blue = count_mask(mask_blue[90:,:])
    count, w, h, x_min, x_max = blue
    if count == 0 or w > 20 or h > 20:
        return 0, None, None, None, None
#    print(count)
    mask_orange = np.logical_and(r > 40, np.logical_and(r/1.5 > g, r/2 > b))
    orange = count_mask(mask_orange)

#    not_mask = np.logical_not(mask_orange)
#    img2 = img.copy()
#    img2[not_mask] = (0, 0, 0)
#    img2[mask_orange] = (0, 128, 255)
#    img2[mask_blue] = (255, 0, 0)
#    cv2.imwrite('artf.jpg', img2)

    mask = np.logical_or(mask_orange, mask_blue)
    return blue  #count_mask(mask)


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
        self.best_depth = None
        self.verbose = False
        self.dump_dir = None  # optional debug ouput into directory
        self.scan = None  # should laster initialize super()
        self.depth = None  # more precise definiton of depth image
        self.width = None  # detect from incoming images

        self.gas_best = None
        self.gas_best_count = 0

    def handle_gas_artifact(self, data):
        print(self.time, 'Gas detected', data)
        if data:  # in virtual world only Boolean is used and transition is reported
            deg_100th, dist_mm = 0, 0  # first approximation without scan and entrance detection
            self.publish('artf', [GAS, deg_100th, dist_mm])

    def waitForImage(self):
        channel = ""
        while channel != "image":
            self.time, channel, data = self.listen()
            setattr(self, channel, data)

            # handling special artifact "gas", which is not requiring image
            if channel == "gas_detected":
                self.handle_gas_artifact(data)
            elif channel == "co2":
                if self.gas_best is None or self.gas_best < data:
                    self.gas_best = data
                    self.gas_best_count = 10
                    print(self.time, 'GAS CO2 value', self.gas_best)

                if self.gas_best > CO2_REPORT_LIMIT and self.gas_best_count == 0:
                    self.handle_gas_artifact(True)
                    self.gas_best = None

                if self.gas_best_count > 0:
                    self.gas_best_count -= 1
            elif channel == "wifiscan":
               for name, signal in data:
                    if name.startswith(PHONE_PREFIX) and signal > PHONE_SIGNAL_LIMIT:
                        deg_100th, dist_mm = 0, 0  # approximation
                        self.publish('artf', [PHONE, deg_100th, dist_mm])

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
        phone_count, w, h, x_min, x_max = count_orange_blue(img)
        if phone_count >= 25:
            print(self.time, 'phone', phone_count)
            artf = PHONE
            deg_100th, dist_mm = 0, 500  # in front of the robot
            self.publish('artf', [artf, deg_100th, dist_mm])
            self.publish('debug_artf', image)  # JPEG
            if self.dump_dir is not None:
                filename = 'artf_%s_%d.jpg' % (artf, self.time.total_seconds())
                with open(os.path.join(self.dump_dir, filename), 'wb') as f:
                    f.write(image)
        rcount, w, h, x_min, x_max = count_red(img)
        yellow_used = False
        if True and rcount < 20:
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
        if count > RED_THRESHOLD or (yellow_used and count > YELLOW_THRESHOLD):
            if self.best is None or count > self.best:
                self.best = count
                self.best_count = 10
                self.best_img = self.image
                self.best_info = w, h, x_min, x_max, (count == rcount), yellow_used  # RED used
                self.best_scan = self.scan
                self.best_depth = self.depth

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
                    self.best_depth = None
                    return
                self.stdout(rcount, w, h, x_min, x_max, w/h, count/(w*h))

            if red_used or yellow_used:
                if self.best_depth is not None:
                    global g_mask
                    g_mask = None
                    img = cv2.imdecode(np.fromstring(self.best_img, dtype=np.uint8), 1)
                    if red_used:
                        count_red(img)
                    else:
                        count_yellow(img)
                    assert g_mask is not None  # intermediate results
                    dist_mm = int(np.median(self.best_depth[g_mask]))
                    mask2 = np.abs(self.best_depth - dist_mm) < 200
                    mask = np.logical_and(g_mask, mask2)
                    # debug
                    #img2 = img.copy()
                    #img2[:] = (0, 0, 0)
                    #img2[g_mask] = (0, 0, 255)
                    #img2[mask] = (0, 255, 0)
                    #cv2.imwrite('artf.jpg', img2)

                    count, w, h, x_min, x_max = count_mask(mask)
                    FX = 462.1  # Focal length.
                    if (dist_mm > 10000 or count < RED_YELLOW_MIN_3D_THRESHOLD or # mix of infinity
                            (red_used and dist_mm * h/FX < 210) or                # robot
                            (yellow_used and dist_mm * h/FX > 850)):              # too high for survivor
                        self.stdout('Invalid distance, ignore, count=', self.best, count, dist_mm, dist_mm * h/FX)
                        # reset detector
                        self.best = None
                        self.best_count = 0
                        self.best_img = None
                        self.best_info = None
                        self.best_scan = None
                        self.best_depth = None
                        return

                    deg_100th = int(round(100 * 69.4 * (self.width/2 - (x_min + x_max)/2)/self.width))

#                    img2 = img.copy()
#                    img2[mask] = (0, 0, 255)
#                    img2[np.logical_not(mask)] = (0, 0, 0)
#                    cv2.imwrite('artfX.jpg', img2)
                else:
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
                if self.best > YELLOW_MAX_THRESHOLD:
                    # too much yellow - barrel or yellow machine
                    self.best = None
                    self.best_count = 0
                    self.best_img = None
                    self.best_info = None
                    self.best_scan = None
                    self.best_depth = None
                    return
                artf = RESCUE_RANDY  # used to be RADIO
            self.stdout(self.time, 'Relative position:', self.best, deg_100th, dist_mm, artf)

            dx_mm, dy_mm = 0, 0  # relative offset to current robot position
            # TODO if VALVE -> find it in scan
            self.publish('artf', [artf, deg_100th, dist_mm])
            self.publish('debug_artf', self.best_img)
            if self.dump_dir is not None:
                filename = 'artf_%s_%d.jpg' % (artf, self.time.total_seconds())
                with open(os.path.join(self.dump_dir, filename), 'wb') as f:
                    f.write(self.best_img)
                if self.best_depth is not None:
                    filename = 'artf_%s_%d.npz' % (artf, self.time.total_seconds())
                    np.savez_compressed(os.path.join(self.dump_dir, filename), depth=self.best_depth)

            # reset detector
            self.best = None
            self.best_count = 0
            self.best_img = None
            self.best_info = None
            self.best_scan = None
            self.best_depth = None


class ArtifactReporter(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register("artf_cmd")
        self.repeat_report_sec = config.get('repeat_report_sec')

    def update(self):  # hack, this method should be called run instead!
        channel = super().update()  # define self.time
        assert channel in ["artf_xyz", "sim_time_sec"], channel

        if channel == 'sim_time_sec':
            if self.repeat_report_sec is None or self.sim_time_sec % self.repeat_report_sec != 0:
                return channel

        print(self.time, "DETECTED:")
        for artf_type, ix, iy, iz in self.artf_xyz:
            print(" ", artf_type, ix/1000.0, iy/1000.0, iz/1000.0)
            s = '%s %.2f %.2f %.2f\n' % (artf_type, ix/1000.0, iy/1000.0, iz/1000.0)
            self.publish('artf_cmd', bytes('artf ' + s, encoding='ascii'))

        print('report completed')
        return channel


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
    parser.add_argument('--depth', help='filename of depth image for tested together with JPEG image')
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
    tester.register('scan', 'image', 'tick', 'depth')
    bus.connect('tester.scan', 'detector.scan')
    bus.connect('tester.depth', 'detector.depth')
    bus.connect('tester.image', 'detector.image')
    bus.connect('detector.artf', 'tester.artf')
    bus.connect('tester.tick', 'tester.tick')
    bus.connect('detector.dropped', 'tester.dropped')
    tester.publish('scan', [2000]*270)  # pretend that everything is at 2 meters
    if args.depth is not None or args.filename.endswith('.npz'):
        filename = args.depth if args.depth is not None else args.filename
        with np.load(filename) as f:
            depth = f['depth']
            tester.publish('depth', depth)
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
