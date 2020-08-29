"""
  Light Manager
"""

import cv2
import numpy as np

from moon.moonnode import MoonNode

class LightManager(MoonNode):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        self.light_intensity = 0.0

    def on_left_image(self, data):
        limg = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
        CAMERA_HEIGHT,CAMERA_WIDTH, _ = limg.shape
        hsv = cv2.cvtColor(limg, cv2.COLOR_BGR2HSV)
        mask = np.zeros((CAMERA_HEIGHT,CAMERA_WIDTH), np.uint8)
        circle_mask = cv2.circle(mask,(CAMERA_HEIGHT//2,CAMERA_WIDTH//2),200,(255,255,255),thickness=-1)
        hist = cv2.calcHist([limg],[2],circle_mask,[256],[0,256])
        topthird = hist[170:]
        brightness = int(sum(topthird) / len(topthird))
        if self.debug:
            print (self.sim_time, "LightManager: Current brightness: %d" % brightness)
        if brightness < 300:
            self.light_intensity = min(1.0, self.light_intensity + 0.1)
            self.send_request('set_light_intensity %s' % str(self.light_intensity))
            if self.debug:
                print (self.sim_time, "LightManager: Increasing light to: %.1f" % self.light_intensity)
        elif brightness > 500:
            self.light_intensity = max(0.0, self.light_intensity - 0.1)
            self.send_request('set_light_intensity %s' % str(self.light_intensity))
            if self.debug:
                print (self.sim_time, "LightManager: Lowering light to: %.1f" % self.light_intensity)




# vim: expandtab sw=4 ts=4
