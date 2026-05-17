import math
import cv2
import numpy as np

from osgar.node import Node
from osgar.lib.quaternion import euler_zyx

class ImageRotator(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('image')
        # Configurable roll multiplier. Usually -1 or 1 depending on camera orientation.
        self.roll_multiplier = config.get('roll_multiplier', -1.0)
        self.last_orientation = None

    def on_orientation_list(self, data):
        if len(data) > 0:
            # OAK-D orientation list format: [timestamp, accuracy, i, j, k, real]
            # which maps to quaternion [x, y, z, w] -> indices 2, 3, 4, 5
            self.last_orientation = data[-1][2:6]

    def on_pose3d(self, data):
        # Alternative input from standard pose3d
        # data format is [ [x, y, z], [qx, qy, qz, qw] ]
        if len(data) == 2 and len(data[1]) == 4:
            self.last_orientation = data[1]

    def on_image(self, data):
        if self.last_orientation is None:
            # Pass through the image if no orientation received yet
            self.publish('image', data)
            return

        yaw, pitch, roll = euler_zyx(self.last_orientation)

        # Decode image
        img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_ANYCOLOR)
        if img is None:
            # Failed to decode, just pass through
            self.publish('image', data)
            return
            
        h, w = img.shape[:2]
        center = (w / 2, h / 2)
        
        # Calculate rotation angle in degrees
        angle_rad = roll * self.roll_multiplier
        angle_deg = math.degrees(angle_rad)
        
        # Rotate image to compensate roll
        M = cv2.getRotationMatrix2D(center, angle_deg, 1.0)
        rotated = cv2.warpAffine(img, M, (w, h))

        # Encode back to JPEG
        _, encoded_img = cv2.imencode('.jpeg', rotated)
        self.publish('image', encoded_img.tobytes())

# vim: expandtab sw=4 ts=4
