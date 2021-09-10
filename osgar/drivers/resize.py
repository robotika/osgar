import cv2
import numpy as np

from osgar.node import Node

class Resize(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('image', 'depth')
        self.width = config['width']
        self.height = config['height']

    def update(self):
        timestamp, channel, data = self.bus.listen()
    
        if channel == 'depth':
            small_data = cv2.resize(data, (self.width, self.height))
        elif channel == 'image':
            img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_ANYCOLOR)
            small_img = cv2.resize(img, (self.width, self.height))
            small_data = cv2.imencode('.jpeg', small_img)[1].tobytes()
        self.publish(channel, small_data)
