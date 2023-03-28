import cv2
import numpy as np

from osgar.node import Node

class Resize(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('image', 'depth')
        self.width = config['width']
        self.height = config['height']

    def on_depth(self, data):
        small_data = cv2.resize(data, (self.width, self.height))
        self.publish('depth', small_data)

    def on_image(self, data):
        img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_ANYCOLOR)
        small_img = cv2.resize(img, (self.width, self.height))
        small_data = cv2.imencode('.jpeg', small_img)[1].tobytes()
        self.publish('image', small_data)
