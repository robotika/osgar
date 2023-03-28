"""
  Wrapper of desired speed for ROS Twist
"""
import math

from osgar.node import Node


class TwistWrap(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register("cmd_vel")

    def on_desired_speed(self, data):
        linear = [data[0]/1000, 0.0, 0.0]
        angular = [0.0, 0.0, math.radians(data[1]/100.0)]
        self.publish('cmd_vel', [linear, angular])

# vim: expandtab sw=4 ts=4
