"""
  Example of robot diver outside OSGAR package
    - simulation of differential robot
"""
import math

from osgar.node import Node
from osgar.bus import BusShutdownException


class MyTimer(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('tick')
        self.sleep_time = config['sleep']

    def run(self):
        try:
            while self.is_bus_alive():
                self.publish('tick', None)
                self.sleep(self.sleep_time)

        except BusShutdownException:
            pass


class MyRobot(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('pose2d', 'emergency_stop')
        self.pose = (0, 0, 0)
        self.speed, self.angular_speed = 0, 0
        self.desired_speed, self.desired_angular_speed = 0, 0
        self.last_update = None

    def send_pose(self):
        x, y, heading = self.pose        
        self.publish('pose2d', [round(x*1000), round(y*1000),
                                round(math.degrees(heading)*100)])

    def update_pose(self):
        if self.last_update is not None:
            t = (self.time - self.last_update).total_seconds()
            self.speed = self.desired_speed  # TODO motion model
            self.angular_speed = self.desired_angular_speed

            x, y, heading = self.pose
            x += math.cos(heading) * self.speed * t
            y += math.sin(heading) * self.speed * t
            heading += self.angular_speed * t
            self.pose = (x, y, heading)
        self.last_update = self.time

    def on_desired_speed(self, data):
        self.update_pose()
        self.desired_speed, self.desired_angular_speed = data[0]/1000.0, math.radians(data[1]/100.0)
        self.send_pose()

    def on_tick(self, data):
        self.update_pose()
        self.send_pose()

# vim: expandtab sw=4 ts=4
