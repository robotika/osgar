"""
  Detect SubT Artifact in Camera Image
"""

import cv2
import numpy as np

from osgar.node import Node


class ArtifactDetector(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)

    def update(self):  # hack, this method should be called run instead!       
        channel = super().update()  # define self.time
        assert channel == "image", channel
        img = cv2.imdecode(np.fromstring(self.image, dtype=np.uint8), 1)
        print(self.time, img.shape)

# vim: expandtab sw=4 ts=4

