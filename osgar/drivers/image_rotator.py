import math
import cv2
import numpy as np

from osgar.node import Node
from osgar.lib.quaternion import euler_zyx

class ImageRotator(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('image', 'rotation_angle')
        self.method = config.get('method', 'imu') # 'imu' or 'optical_flow'
        self.roll_multiplier = config.get('roll_multiplier', -1.0)
        self.verbose = False  # TO be removed after master merge
        
        # IMU state
        self.last_orientation = None
        
        # Optical flow state
        self.prev_gray = None
        self.accumulated_angle = 0.0

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

    def _get_optical_flow_angle(self, img):
        curr_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        if self.prev_gray is None:
            self.prev_gray = curr_gray
            return 0.0
            
        prev_pts = cv2.goodFeaturesToTrack(self.prev_gray,
                                           maxCorners=200,
                                           qualityLevel=0.01,
                                           minDistance=30,
                                           blockSize=3)
                                           
        if prev_pts is not None and len(prev_pts) > 0:
            curr_pts, status, err = cv2.calcOpticalFlowPyrLK(self.prev_gray, curr_gray, prev_pts, None)
            idx = np.where(status == 1)[0]
            if len(idx) > 0:
                prev_pts = prev_pts[idx]
                curr_pts = curr_pts[idx]
                
                m, inliers = cv2.estimateAffinePartial2D(prev_pts, curr_pts)
                if m is not None:
                    # m[1, 0] is sin(da), m[0, 0] is cos(da)
                    da = np.arctan2(m[1, 0], m[0, 0])
                    self.accumulated_angle += da
        
        self.prev_gray = curr_gray
        return self.accumulated_angle

    def on_image(self, data):
        img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_ANYCOLOR)
        if img is None:
            self.publish('image', data)
            return

        angle_rad = 0.0
        if self.method == 'imu':
            if self.last_orientation is None:
                self.publish('image', data)
                return
            yaw, pitch, roll = euler_zyx(self.last_orientation)
            if self.verbose:
                print(f'yaw = {math.degrees(yaw)}, pitch = {math.degrees(pitch)}, roll = {math.degrees(roll)}')
            angle_rad = roll * self.roll_multiplier
        elif self.method == 'optical_flow':
            angle_rad = self._get_optical_flow_angle(img) * self.roll_multiplier

        self.publish('rotation_angle', angle_rad)

        h, w = img.shape[:2]
        center = (w / 2, h / 2)
        angle_deg = math.degrees(angle_rad)
        
        M = cv2.getRotationMatrix2D(center, angle_deg, 1.0)
        rotated = cv2.warpAffine(img, M, (w, h))

        _, encoded_img = cv2.imencode('.jpeg', rotated)
        self.publish('image', encoded_img.tobytes())

# vim: expandtab sw=4 ts=4
