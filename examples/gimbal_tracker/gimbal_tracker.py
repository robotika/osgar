"""
zdroj videa (OSGAR "opencv" Node)
- Detector - vstup snimky, vyber nejnovejsi, detekce, publikuj cas prichoziho snimku a pole bboxu
- Servo - dostavej par integeru odpovidajici raw PWM
- Controller, sbira info o bboxech, pripadne si trackuje ocekavanou polohu serv v case a vysila prika
"""

from osgar.node import Node


class Detector(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('bbox')


class Controller(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('desired_position')

    def on_position(self, data):
        pass


class Servo(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('position')
        self.position = 0
    def on_tick(self, data):
        # TODO move commands
        self.publish('position', self.position)
