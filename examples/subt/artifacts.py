"""
  Detect SubT Artifact in Camera Image
"""

import cv2
import numpy as np

from osgar.node import Node

def count_red(img):
    count = 0
    for x in range(320):
        for y in range(240):
            b, g, r = img[y][x]
            if r > 100 and r > 2 * g and r > 2 * b:
                count += 1
    return count


class ArtifactDetector(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)

    def update(self):  # hack, this method should be called run instead!
        channel = super().update()  # define self.time
        assert channel == "image", channel
        img = cv2.imdecode(np.fromstring(self.image, dtype=np.uint8), 1)
        count = count_red(img)
        print(self.time, img.shape, count)
        if count > 100:
            self.publish('artf', count)

# vim: expandtab sw=4 ts=4

