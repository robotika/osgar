"""
zdroj videa (OSGAR "opencv" Node)
- Detector - vstup snimky, vyber nejnovejsi, detekce, publikuj cas prichoziho snimku a pole bboxu
- Servo - dostavej par integeru odpovidajici raw PWM
- Controller, sbira info o bboxech, pripadne si trackuje ocekavanou polohu serv v case a vysila prika
"""

from osgar.node import Node

class Detector(Node):
    pass


class Servo(Node):
    pass


class Controller(Node):
    pass
