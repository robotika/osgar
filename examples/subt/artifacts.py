"""
  Detect SubT Artifact in Camera Image
"""
from datetime import timedelta

import cv2
import numpy as np

from osgar.node import Node

EXTINGUISHER = 'TYPE_EXTINGUISHER'


def old_count_red(img):
    count = 0
    for x in range(320):
        for y in range(240):
            b, g, r = img[y][x]
            if r > 100 and r > 2 * g and r > 2 * b:
                count += 1
    return count


def count_red(img):
    b = img[:,:,0]
    g = img[:,:,1]
    r = img[:,:,2]
    mask = np.logical_and(r > 100, np.logical_and(r/2 > g, r/2 > b))
    img[mask] = 0, 255, 0
    return int(mask.sum())


class ArtifactDetector(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        self.best = None
        self.best_count = 0
        self.active = True

    def update(self):  # hack, this method should be called run instead!
        channel = super().update()  # define self.time
        assert channel == "image", channel
        if not self.active:
            return channel

        # hack - test communication to BaseStation
        if self.time > timedelta(minutes=4):
            print('Published', self.best)
            self.publish('artf', EXTINGUISHER)
            self.active = False
        return channel
        # END OF HACK ....

        img = cv2.imdecode(np.fromstring(self.image, dtype=np.uint8), 1)
        count = count_red(img)
#        print(self.time, img.shape, count)
        if self.best_count > 0:
            self.best_count -= 1
        if self.best is None:
            if count > 100:
                self.best = count
                self.best_count = 10
        elif count > self.best:
            self.best = count
            self.best_count = 10

        if self.best is not None and self.best_count == 0:
            print('Published', self.best)
            self.publish('artf', EXTINGUISHER)
            self.active = False
        return channel


class ArtifactReporter(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)

    def update(self):  # hack, this method should be called run instead!
        channel = super().update()  # define self.time
        assert channel == "artf_xyz", channel

        print("DETECTED", self.artf_xyz)

        artf_type, ix, iy, iz = self.artf_xyz
        # TODO call SubT API

        with open('call_base.txt', 'w') as f:
            f.write('%s %.2f %.2f %.2f\n' % (artf_type, ix/1000.0, iy/1000.0, iz/1000.0))
        print('report completed')

# vim: expandtab sw=4 ts=4

